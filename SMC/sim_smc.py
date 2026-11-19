# =============================================================================
# sim_smc.py - Simulador del AUTO LEGO: PI de main_rs.py vs SMC (smc_control.py)
# =============================================================================
# Reproduce el lazo del firmware (dt = 1 ms, referencia Euler con N entero de
# ticks, cambio de tramo por posicion medida, PI sin antiwindup, ganancias de
# controlConstants.py) sobre un modelo cinematico tipo bicicleta con actuadores
# de 1er orden y sensores no ideales. Usa el planner REAL (reeds_shepp.py).
#
# Uso:   python3 SMC/sim_smc.py           (corre todo, guarda figs/ y metricas)
# =============================================================================
import sys, math, os, copy
HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.modules['umath'] = math            # el planner importa umath (MicroPython)
sys.path.insert(0, REPO)
sys.path.insert(0, HERE)
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from reeds_shepp import get_best_path, normalize_start_and_end_point, denormalize_distance_in_path
from smc_control import LateralSMC, SpeedSMC

# ---------------------------------------------------------------------------
# Parametros del firmware (controlConstants.py / vehicleConstants.py)
# ---------------------------------------------------------------------------
L_CAR = 24.0
PSI_MAX = 20.0
R_TURN = 80.0
VELOCITY = 17.0                            # [cm/s] velocidad REAL (el PI corria a ~17 con lazo libre de 2.34 ms)
KP_POS, KI_POS, KP_ST, KI_ST = 50.0, 0.01, 50.0, 0.01
ANGLE_TO_CM = 0.045
H_MS = 5                                   # periodo FIJO del lazo (LOOP_MS del firmware)
KI_REF_MS = 2.34                           # periodo con el que se sintonizo el Ki por tick del PI
DT = H_MS / 1000.0

# ---------------------------------------------------------------------------
# Parametros de la PLANTA (SUPUESTOS: reemplazar por los identificados, ver .md)
# ---------------------------------------------------------------------------
PLANT = dict(
    Kv=0.8,            # [cm/s por % de potencia]  (80 cm/s a 100%)
    tau_v=0.15,        # [s]   constante de tiempo traccion
    tau_s=0.06,        # [s]   constante de tiempo servo de direccion
    rate_s=math.radians(120.0),   # [rad/s] limite de velocidad del servo
    psi_max_true=PSI_MAX,         # [deg] angulo de rueda a +-100 %
    trim_deg=0.0,      # [deg] desalineacion mecanica de la direccion
    dKv=0.0,           # error relativo de ganancia de traccion (bateria)
    enc_err=0.0,       # error relativo de escala de encoders (patinaje)
    gyro_drift=0.0,    # [deg/s] deriva del IMU
    yaw_kick=None,     # (t0, t1, w [rad/s]) perturbacion de guinada (patinaje)
    x0=0.0, y0=0.0, th0_deg=0.0,  # pose inicial verdadera (math) respecto de la ref
    est_knows_offset=False,       # el estimador arranca con el offset (pose externa)
    sigma_imu=0.05, sigma_v=1.0,
    seed=1,
)

# Sintonia SMC (ver SMC_diseno.md, secciones 4 y 5)
SMC_LAT = dict(lam=0.035, eta=0.008, phi=0.05, ki=0.0)
T_LEAD = 0.20        # [s] anticipacion del feedforward en las transiciones de tramo
SMC_SPD = dict(K=0.8, tau=0.15, c=5.0, eta=20.0, phi=8.0)


def plan(end_pose, r=R_TURN):
    sp, ep = normalize_start_and_end_point([0, 0, 0], end_pose, r)
    best = get_best_path(sp, ep)
    leg = denormalize_distance_in_path(best, r)
    return [s for s in leg if s['distance'] >= 1.0]       # MIN_SEG_DISTANCE


def dense_reference(leg, step=0.2):
    """Camino geometrico de referencia (misma integracion que stream_plan)."""
    x = y = th = 0.0
    pts = [(0.0, 0.0)]
    for seg in leg:
        n = max(1, int(seg['distance'] / step))
        ds = seg['distance'] / n
        for _ in range(n):
            th -= seg['gear'] * seg['steering'] * ds / R_TURN
            x += seg['gear'] * ds * math.cos(th)
            y += seg['gear'] * ds * math.sin(th)
            pts.append((x, y))
    return np.array(pts), th


def sat_i(v, lim):
    return int(min(max(v, -lim), lim))


def run(leg, ctrl, P, lat=None, spd=None, tmax=30.0, t_lead=0.0):
    """ctrl: 'PI' (firmware) | 'SMC' (direccion SMC + traccion PI) | 'SMC2'
    (direccion SMC + traccion SMC). Devuelve dict con logs y metricas."""
    p = copy.deepcopy(PLANT); p.update(P)
    lat = lat or SMC_LAT
    spd = spd or SMC_SPD
    rng = np.random.RandomState(p['seed'])
    smc_l = LateralSMC(L_CAR, PSI_MAX, **lat)
    smc_v = SpeedSMC(**spd)
    dmax_true = math.radians(p['psi_max_true'])

    # estado verdadero (math)
    X, Y, TH = p['x0'], p['y0'], math.radians(p['th0_deg'])
    pos_true = 0.0
    v = 0.0
    delta = 0.0
    vf = 0.0
    t = 0.0
    # estimador (odometria del firmware: encoders + IMU)
    if p['est_knows_offset']:
        xh, yh, th_imu0 = p['x0'], p['y0'], math.radians(p['th0_deg'])
    else:
        xh, yh, th_imu0 = 0.0, 0.0, 0.0            # resetAllSensors(): IMU=0
    th_off = TH - th_imu0                          # TH = th_imu_math + th_off (+deriva)

    steering_reference = 0.0   # rad, compass (CW+) como en main_rs.py
    xr = yr = 0.0
    log = {k: [] for k in ('t', 'x', 'y', 'th', 'u_st', 'u_dr', 'v', 'ey', 'et', 's', 'seg')}
    elapsed_ms = 0
    status = 'ok'
    pos_prev_true_seg = 0.0
    end_pose = None

    for iseg, seg in enumerate(leg):
        distance, steer_dir, gear = seg['distance'], seg['steering'], seg['gear']
        delta_s = H_MS * VELOCITY / 1000.0
        N_seg = int(distance / delta_s)
        if N_seg * delta_s < distance:
            N_seg += 1
        N_seg = max(N_seg, 1)
        ds_seg = distance / N_seg
        ref_ticks = 0
        head0 = steering_reference
        pos_base = pos_true                              # front/rear.reset_angle(0)
        position_reference = 0.0
        position_vehicle = 0.0
        position_vehicle_prev = 0.0
        steering_I = 0.0
        position_I = 0.0
        smc_l.reset(); smc_v.reset()
        kappa_ref = -steer_dir / R_TURN
        nxt = leg[iseg + 1] if iseg + 1 < len(leg) else None
        v_ref = gear * ds_seg / DT

        while abs(position_vehicle) <= distance and elapsed_ms / 1000.0 < tmax:
            # ---- referencia (Euler N entero, snap en el ultimo tick) ----
            if ref_ticks < N_seg:
                ref_ticks += 1
                if ref_ticks < N_seg:
                    position_reference += gear * ds_seg
                    steering_reference += gear * steer_dir * ds_seg / R_TURN
                else:
                    position_reference = gear * distance
                    steering_reference = head0 + gear * steer_dir * distance / R_TURN
                thr = -steering_reference
                xr += gear * ds_seg * math.cos(thr)
                yr += gear * ds_seg * math.sin(thr)
            thr = -steering_reference

            # ---- sensores ----
            enc = (pos_true - pos_base) * (1.0 + p['enc_err'])
            position_vehicle = round(enc / ANGLE_TO_CM) * ANGLE_TO_CM
            th_imu_math = TH - th_off - math.radians(p['gyro_drift']) * (-1.0) * t * 0 \
                + math.radians(p['gyro_drift']) * t + rng.normal(0, math.radians(p['sigma_imu']))
            steering_vehicle = -math.degrees(th_imu_math)           # compass (CW+)
            vf += (v * (1.0 + p['enc_err']) - vf) * (DT / 0.02)
            v_meas = vf + rng.normal(0, p['sigma_v'])

            # ---- errores ----
            position_error = position_reference - position_vehicle
            steering_error = math.degrees(steering_reference) - steering_vehicle   # deg
            ds_m = position_vehicle - position_vehicle_prev
            position_vehicle_prev = position_vehicle
            hr = -math.radians(steering_vehicle)
            xh += ds_m * math.cos(hr)
            yh += ds_m * math.sin(hr)

            # ---- comando de TRACCION ----
            if ctrl == 'SMC2':
                u_dr = smc_v.update(position_error, v_ref, v_meas)
                u_dr = sat_i(u_dr, 100)
            else:
                position_P = KP_POS * position_error
                position_I += KI_POS * (H_MS / KI_REF_MS) * position_error
                u_dr = sat_i(position_P + position_I, 100)

            # ---- curvatura anticipada (preview) para el feedforward ----
            kappa_lead = kappa_ref
            if nxt is not None and nxt['gear'] == gear and t_lead > 0.0 and \
                    distance - abs(position_vehicle) < VELOCITY * t_lead:
                kappa_lead = -nxt['steering'] / R_TURN

            # ---- comando de DIRECCION ----
            if ctrl in ('PI', 'PIFF'):
                steering_P = KP_ST * steering_error
                steering_I += KI_ST * (H_MS / KI_REF_MS) * steering_error
                u_st = gear * sat_i(steering_P + steering_I, 100)
                if ctrl == 'PIFF':        # feedforward: delta = atan(L*kappa) en % (compass: signo -)
                    ff = 100.0 * math.atan(L_CAR * kappa_lead) / math.radians(PSI_MAX)
                    u_st = int(max(-100, min(100, u_st - ff)))
                ey = s_log = 0.0
                et = math.radians(steering_error)
            else:
                dx, dy = xh - xr, yh - yr
                e_s = math.cos(thr) * dx + math.sin(thr) * dy
                ey = -math.sin(thr) * dx + math.cos(thr) * dy
                et = math.radians(steering_error)      # e_theta_math == steering_error
                pct = smc_l.update(gear, et, ey, e_s, kappa_ref, abs(ds_m), kappa_lead)
                u_st = int(round(-pct))                # firmware: compass CW+ -> signo menos
                s_log = smc_l.s

            # ---- planta (Euler semi-implicito, dt = 1 ms) ----
            Kv = p['Kv'] * (1.0 + p['dKv'])
            v += (Kv * u_dr - v) / p['tau_v'] * DT
            d_cmd = -u_st / 100.0 * dmax_true + math.radians(p['trim_deg'])
            dd = (d_cmd - delta) / p['tau_s']
            dd = max(-p['rate_s'], min(p['rate_s'], dd))
            delta += dd * DT
            w = v * math.tan(delta) / L_CAR
            if p['yaw_kick'] and p['yaw_kick'][0] <= t < p['yaw_kick'][1]:
                w += p['yaw_kick'][2]
            TH += w * DT
            X += v * math.cos(TH) * DT
            Y += v * math.sin(TH) * DT
            pos_true += v * DT
            t += DT
            elapsed_ms += H_MS

            if int(round(t / DT)) % 5 == 0:
                for k, val in (('t', t), ('x', X), ('y', Y), ('th', TH), ('u_st', u_st),
                               ('u_dr', u_dr), ('v', v), ('ey', ey), ('et', et),
                               ('s', s_log), ('seg', iseg)):
                    log[k].append(val)
        if elapsed_ms / 1000.0 >= tmax:
            status = 'timeout'
            break
        end_pose = (X, Y, TH)       # pose verdadera al cruzar el final del tramo
    return dict(log={k: np.array(v) for k, v in log.items()}, status=status,
                end_pose=end_pose, ref_end=(xr, yr, -steering_reference))


def metrics(res, ref_pts, t_late=2.5):
    L = res['log']
    P = np.stack([L['x'], L['y']], 1)
    d = np.sqrt(((P[:, None, :] - ref_pts[None, ::3, :]) ** 2).sum(2)).min(1)   # dist al camino
    late = d[L['t'] > t_late]
    u = L['u_st'].astype(float)
    tv = np.abs(np.diff(u)).sum() / max(L['t'][-1], 1e-9)                       # [% / s]
    xr, yr, thr = res['ref_end']
    dx, dy = res['end_pose'][0] - xr, res['end_pose'][1] - yr
    end_lat = -math.sin(thr) * dx + math.cos(thr) * dy
    end_th = math.degrees(res['end_pose'][2] - thr)
    return dict(d_rms=float(np.sqrt((d ** 2).mean())), d_max=float(d.max()),
                d_late=float(np.sqrt((late ** 2).mean())), tv=float(tv),
                end_lat=float(end_lat), end_th=float(end_th),
                T=float(L['t'][-1]), status=res['status'], d=d)


SCENARIOS = [
    ('S1 nominal',                   dict()),
    ('S2 offset inicial 10cm+15deg', dict(y0=10.0, th0_deg=15.0, est_knows_offset=True)),
    ('S3 planta incierta',           dict(psi_max_true=18.0, trim_deg=1.0, dKv=-0.30,
                                          enc_err=0.03, gyro_drift=0.1)),
    ('S4 patinaje (kick guinada)',   dict(yaw_kick=(2.0, 2.5, 0.25))),
]
CONFIGS = [('PI', 'A: PI+PI (firmware)'), ('PIFF', 'B: PI + FF/preview'),
           ('SMC', 'C: SMC dir + PI tracc'), ('SMC2', 'D: SMC dir + SMC tracc')]


def main(end_pose=(120.0, 60.0, -90.0), tag=''):
    leg = plan(list(end_pose))
    ref_pts, th_end = dense_reference(leg)
    print('Plan RS {} ->'.format(end_pose), [(s['steering'], s['gear'], round(s['distance'], 1)) for s in leg])
    print('Referencia termina en x={:.1f} y={:.1f} th={:.1f} deg (convencion math del dashboard)'.format(
        ref_pts[-1, 0], ref_pts[-1, 1], math.degrees(th_end)))
    out = {}
    lines = []
    hdr = '{:<30s}{:<24s}{:>7s}{:>7s}{:>8s}{:>9s}{:>9s}{:>9s}{:>6s}  {}'.format(
        'escenario', 'config', 'd_rms', 'd_max', 'd_late', 'TV[%/s]', 'fin_lat', 'fin_th', 'T[s]', 'estado')
    lines.append(hdr)
    for sname, sp in SCENARIOS:
        for c, cname in CONFIGS:
            r = run(leg, c, sp, t_lead=T_LEAD)
            m = metrics(r, ref_pts)
            out[(sname, c)] = (r, m)
            lines.append('{:<30s}{:<24s}{:7.2f}{:7.2f}{:8.2f}{:9.0f}{:9.2f}{:9.2f}{:6.1f}  {}'.format(
                sname, cname, m['d_rms'], m['d_max'], m['d_late'], m['tv'], m['end_lat'], m['end_th'], m['T'], m['status']))
    print('\n'.join(lines))
    with open(os.path.join(HERE, 'figs', 'metricas{}.txt'.format(tag)), 'w') as f:
        f.write('d_*: distancia verdadera al camino [cm] (d_late: RMS para t>2.5 s); TV: variacion total del\n'
                'comando de direccion [%/s]; fin_lat/fin_th: error LATERAL [cm] y de rumbo [deg] de la pose\n'
                'verdadera al cruzar el final del ultimo tramo, en el marco de la referencia\n\n')
        f.write('\n'.join(lines) + '\n')

    # ---- figuras ----
    colors = {'PI': '#d62728', 'PIFF': '#ff7f0e', 'SMC': '#1f77b4', 'SMC2': '#2ca02c'}
    for sname, sp in SCENARIOS:
        fig, ax = plt.subplots(1, 3, figsize=(17, 4.6))
        ax[0].plot(ref_pts[:, 0], ref_pts[:, 1], 'k--', lw=1.2, label='referencia')
        for c, cname in CONFIGS:
            r, m = out[(sname, c)]
            ax[0].plot(r['log']['x'], r['log']['y'], color=colors[c], lw=1.2, label=cname)
            ax[1].plot(r['log']['t'], m['d'], color=colors[c], lw=1.2, label=cname)
            ax[2].plot(r['log']['t'], r['log']['u_st'], color=colors[c], lw=0.7, label=cname)
        ax[0].set_aspect('equal'); ax[0].set_title(sname + ' - trayectoria'); ax[0].set_xlabel('x [cm]'); ax[0].set_ylabel('y [cm]')
        ax[1].set_title('distancia verdadera al camino [cm]'); ax[1].set_xlabel('t [s]'); ax[1].grid(alpha=.3)
        ax[2].set_title('comando de direccion [%]'); ax[2].set_xlabel('t [s]'); ax[2].grid(alpha=.3)
        ax[0].legend(fontsize=8); ax[0].grid(alpha=.3)
        fig.tight_layout()
        fn = os.path.join(HERE, 'figs', 'sim_{}{}.png'.format(sname.split()[0], tag))
        fig.savefig(fn, dpi=110); plt.close(fig)
    return out


def sensitivity(end_pose=(120.0, 60.0, -90.0)):
    """Barrido de la constante de tiempo del servo de direccion (lo menos conocido)."""
    leg = plan(list(end_pose)); ref_pts, _ = dense_reference(leg)
    sp3 = dict(SCENARIOS)['S3 planta incierta']
    lines = ['tau_s[s]  config     S1 d_rms  S1 TV   S3 d_rms  S3 d_max  S3 TV']
    for tau in (0.03, 0.06, 0.12, 0.20):
        for c in ('PI', 'PIFF', 'SMC'):
            m1 = metrics(run(leg, c, dict(tau_s=tau), t_lead=T_LEAD), ref_pts)
            m3 = metrics(run(leg, c, dict(sp3, tau_s=tau), t_lead=T_LEAD), ref_pts)
            lines.append('{:7.2f}   {:<9s}{:8.2f}{:8.0f}{:10.2f}{:10.2f}{:7.0f}'.format(
                tau, c, m1['d_rms'], m1['tv'], m3['d_rms'], m3['d_max'], m3['tv']))
    print('\n'.join(lines))
    with open(os.path.join(HERE, 'figs', 'sensibilidad_servo.txt'), 'w') as f:
        f.write('\n'.join(lines) + '\n')


if __name__ == '__main__':
    main()
    sensitivity()
