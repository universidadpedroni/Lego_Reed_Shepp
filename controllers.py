# =============================================================================
# controllers.py - LAZOS DE CONTROL (la "inteligencia" del auto)
# =============================================================================
# Cada tick el lazo principal (main_rs.py) compara donde ESTA el auto con donde
# DEBERIA estar (la referencia) y estos controladores deciden la potencia de los
# motores. Hay dos lazos independientes:
#
#   TRACCION    PI           sigue el arco recorrido de la referencia
#                            (error = arco_ref - arco_auto [cm])
#
#   DIRECCION   SteeringPI   (el mismo PI, otras ganancias) sigue el RUMBO de la referencia
#               SteeringSMC  modo deslizante: sigue rumbo y error lateral a la vez,
#                            con feedforward de curvatura (matematica en smc_control.py)
#
# Los tres tienen la misma interfaz: update(...) -> (comando, parte_P, parte_I)
# donde comando es el porcentaje de potencia ya saturado, y P / I son lo que se
# muestra en las barras del dashboard (en SMC: P = control equivalente, I = conmutado).
#
# Sin hardware: se prueba en la PC con numeros inventados.
# =============================================================================
import umath

from vehicleConstants import max_angle_power, max_drive_power, r_turn_min, L_car
from settings import cfg, LOOP_MS, KI_REF_MS
from smc_control import LateralSMC


def saturate(value, limit):
    """Recorta value a [-limit, limit] y lo devuelve entero (los motores toman
    potencia entera)."""
    return int(min(max(value, -limit), limit))


def integrate(integral, p, delta, limit):
    """Paso de la integral CON ANTIWINDUP. Dos protecciones:
      1. integracion condicional: si el comando YA esta saturado (p + integral fuera de
         [-limit, limit]) y el incremento lo empujaria mas hacia afuera, no se integra.
         Sin esto la integral sigue creciendo mientras el actuador no responde (arranque,
         patinaje, direccion al tope) y despues tarda en descargarse: sobrepico y oscilacion.
      2. tope duro: la integral nunca pasa de +-limit (el comando maximo posible).
    p: parte proporcional actual; delta: incremento de este tick (ki * error)."""
    new = integral + delta
    u = p + new
    if (u > limit and delta > 0) or (u < -limit and delta < 0):
        new = integral
    if new > limit:
        new = limit
    elif new < -limit:
        new = -limit
    return new


class PI:
    """Controlador PI con antiwindup. Una sola clase para los dos lazos: cambian las
    ganancias (claves de cfg.gains) y el tope del comando.
      kp_key, ki_key  claves en cfg.gains: 'kp_pos'/'ki_pos' (traccion), 'kp_st'/'ki_st' (direccion)
      limit           comando maximo [%] (max_drive_power / max_angle_power)
    El Ki esta definido POR TICK a KI_REF_MS; se reescala por h/KI_REF_MS para no
    depender del periodo del lazo. update(error) -> (comando, parte_P, parte_I)."""

    def __init__(self, kp_key, ki_key, limit, h=LOOP_MS):
        self.kp_key = kp_key
        self.ki_key = ki_key
        self.limit = limit
        self.ki_scale = h / KI_REF_MS
        self.integral = 0.0

    def reset(self):
        self.integral = 0.0

    def update(self, error):
        p = cfg.gains[self.kp_key] * error
        self.integral = integrate(self.integral, p,
                                  cfg.gains[self.ki_key] * self.ki_scale * error, self.limit)
        return saturate(p + self.integral, self.limit), p, self.integral


def make_position_controller(h=LOOP_MS):
    """PI de TRACCION sobre el error de arco [cm]."""
    return PI('kp_pos', 'ki_pos', max_drive_power, h)


class SteeringPI:
    """PI de DIRECCION sobre el error de rumbo [grados]: el mismo PI de arriba (con
    las ganancias de direccion) mas la inversion de sentido en marcha atras (gear).
    Tiene la misma interfaz que SteeringSMC para que el lazo principal los intercambie."""

    def __init__(self, h=LOOP_MS):
        self.pi = PI('kp_st', 'ki_st', max_angle_power, h)
        self.s = 0.0           # (solo el SMC lo usa: superficie de deslizamiento)

    def reset(self):
        self.pi.reset()

    def update(self, gear, steering_error, *unused):
        command, p, i = self.pi.update(steering_error)
        return gear * command, p, i


class SteeringSMC:
    """Direccion por modos deslizantes (ver smc_control.py y SMC/SMC_diseno.md).
    Superficie s = e_g + lambda*gear*e_y: junta error de rumbo y error lateral."""

    def __init__(self, *unused):
        sm = cfg.smc
        self.smc = LateralSMC(L_car, sm['psi'], sm['lam'], sm['eta'], sm['phi'], sm['ki'])
        self.s = 0.0

    def reset(self):
        self.smc.reset()

    def update(self, gear, steering_error, e_y, e_s, ds, kappa_ref, k_lead):
        """steering_error en grados; kappa_ref y k_lead en 1/cm (+ = izquierda)."""
        pct = self.smc.update(gear, umath.radians(steering_error), e_y, e_s,
                              kappa_ref, abs(ds), k_lead)
        self.s = self.smc.s
        # El SMC trabaja en convencion matematica (+ = izquierda); el hub usa compass
        # (CW +): de ahi el signo menos.
        command = saturate(-pct, max_angle_power)
        # Para las barras del dashboard (la telemetria las multiplica por gear, por eso
        # se divide aca): P = control equivalente, I = termino conmutado.
        return command, -gear * self.smc.pct_eq, -gear * self.smc.pct_sw


def curvature_preview(nxt, gear, distance, travelled, kappa_ref):
    """Anticipacion de curvatura del SMC: durante los ultimos velocity*tlead cm de un
    segmento, si el siguiente sigue en el mismo sentido, usa su curvatura (el servo
    de direccion tarda y no puede cambiar de curvatura de golpe).
      nxt        segmento siguiente (dict) o None
      travelled  arco ya recorrido en el segmento [cm]"""
    if (nxt is not None and nxt['gear'] == gear and
            distance - travelled < cfg.velocity * cfg.smc['tlead']):
        return -nxt['steering'] / r_turn_min
    return kappa_ref


def make_steering_controller(kind):
    """Devuelve el controlador de direccion elegido ('PI' o 'SMC')."""
    if kind == 'SMC':
        return SteeringSMC()
    return SteeringPI()
