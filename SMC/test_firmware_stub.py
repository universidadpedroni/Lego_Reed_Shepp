# =============================================================================
# test_firmware_stub.py - Corre el main_rs.py REAL (controlDelVehiculo) en CPython
# con stubs de Pybricks y una planta simulada. Verifica que el cableado del firmware
# (signos, orden de odometria, telemetria, selector PI/SMC) anda antes de ir al hub.
#   python3 SMC/test_firmware_stub.py
# =============================================================================
import sys, os, math, io, types, contextlib
HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, REPO)
sys.path.insert(0, HERE)
sys.modules['umath'] = math

# ---- planta (mismos supuestos que sim_smc.py) ----
import sim_smc as S
Pl = dict(S.PLANT)
state = dict(t=0.0, X=0.0, Y=0.0, TH=0.0, v=0.0, delta=0.0, pos=0.0, u_dr=0.0, u_st=0.0, hoff=0.0, base_f=0.0)
log = []

def plant_step(dt):
    s = state
    s['v'] += (Pl['Kv'] * max(-100, min(100, s['u_dr'])) - s['v']) / Pl['tau_v'] * dt
    d_cmd = -s['u_st'] / 100.0 * math.radians(Pl['psi_max_true'])
    dd = max(-Pl['rate_s'], min(Pl['rate_s'], (d_cmd - s['delta']) / Pl['tau_s']))
    s['delta'] += dd * dt
    s['TH'] += s['v'] * math.tan(s['delta']) / S.L_CAR * dt
    s['X'] += s['v'] * math.cos(s['TH']) * dt
    s['Y'] += s['v'] * math.sin(s['TH']) * dt
    s['pos'] += s['v'] * dt
    s['t'] += dt

# ---- stubs de pybricks ----
def mod(name, **kw):
    m = types.ModuleType(name); m.__dict__.update(kw); sys.modules[name] = m; return m

class Motor:
    def __init__(self, *a, **k): pass
    def angle(self): return (state['pos'] / S.ANGLE_TO_CM) - state['base_f']
    def reset_angle(self, a=0): state['base_f'] = state['pos'] / S.ANGLE_TO_CM - a
    def stalled(self): return False
class IMU:
    def heading(self): return -math.degrees(state['TH']) - state['hoff']
    def reset_heading(self, a=0): state['hoff'] = -math.degrees(state['TH']) - a
    def settings(self, *a, **k): pass
class Hub:
    imu = IMU()
    class light:
        @staticmethod
        def on(c): pass
    class battery:
        @staticmethod
        def voltage(): return 8000
        @staticmethod
        def current(): return 100
class Car:
    def __init__(self, st, motors): pass
    def steer(self, p): state['u_st'] = p
    def drive_power(self, p): state['u_dr'] = p
class StopWatch:
    def time(self): return int(state['t'] * 1000)
    def reset(self): pass
def wait(ms):
    plant_step(ms / 1000.0)
    log.append((state['t'], state['X'], state['Y'], state['TH'], state['u_st']))

mod('pybricks'); mod('pybricks.hubs', TechnicHub=lambda: Hub())
mod('pybricks.parameters', Color=types.SimpleNamespace(RED=0, ORANGE=1, GREEN=2), Direction=types.SimpleNamespace(CLOCKWISE=0), Port=types.SimpleNamespace(A=0, B=1, D=3))
mod('pybricks.tools', wait=wait, StopWatch=StopWatch)
mod('pybricks.robotics', Car=Car)
mod('pybricks.pupdevices', Motor=Motor)
class _In:
    class buffer:
        @staticmethod
        def read(n): return b''
mod('usys', stdin=_In())
mod('uselect', poll=lambda: types.SimpleNamespace(register=lambda x: None, poll=lambda t: []))

import numpy as np
import importlib
F = importlib.import_module(os.environ.get('FW_MODULE', 'main_rs'))   # FW_MODULE=main_bundle prueba el archivo unico
F.cfg.velocity = float(os.environ.get('VEL', '17'))   # velocidad real de prueba [cm/s]

def run():
    for k in state: state[k] = 0.0
    log.clear()
    leg = S.plan([120, 60, -90])
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        status = F.controlDelVehiculo(leg, [0, 0, 0], 0, len(leg))
    out = buf.getvalue().splitlines()
    ref, _ = S.dense_reference(leg)
    P = np.array([(l[1], l[2]) for l in log])
    d = np.sqrt(((P[::5, None, :] - ref[None, ::3, :]) ** 2).sum(2)).min(1)
    tl = [l for l in out if l.startswith('T ')]
    nf = len(tl[-1][2:].split(','))
    ey = [float(l[2:].split(',')[15]) for l in tl]
    loop = [l for l in out if l.startswith('LOOP')]
    return status, d, nf, ey, loop, out

ok = True
status, d, nf, ey, loop, out = run()
if os.environ.get('DUMP'):
    open(os.environ['DUMP'] + '_PI.txt', 'w').write('\n'.join(out))
print('PI   estado=%s campos_T=%d d_rms=%.2f d_max=%.2f cm  e_y_estimado rms=%.2f max=%.2f  %s' % (
    status, nf, np.sqrt((d ** 2).mean()), d.max(), np.sqrt(np.mean(np.square(ey))), np.max(np.abs(ey)), loop))
ok &= (status == 'ok' and nf == 17)
print('PRUEBA', 'OK' if ok else 'FALLA')
sys.exit(0 if ok else 1)
