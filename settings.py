# =============================================================================
# settings.py - PARAMETROS AJUSTABLES EN CALIENTE y comandos que los cambian
# =============================================================================
# Todo lo que el dashboard puede modificar entre ensayos vive en UN solo objeto,
# `cfg`. Los demas modulos hacen `from settings import cfg` y leen cfg.velocity,
# cfg.gains, etc. (un objeto compartido: si cambia aca, cambia para todos).
#
# Este archivo NO toca hardware: se puede importar y probar en la PC.
#
# Cada set_xxx(arg) recibe el texto que mando el dashboard (lo que viene despues
# del comando, por ejemplo "40" en "VEL 40"), valida, actualiza cfg y devuelve
# True/False. main_rs.py solo contesta ACK o ERR segun lo que devuelvan.
# =============================================================================
from vehicleConstants import v
from controlConstants import KP_POSITION, KI_POSITION, KP_STEERING, KI_STEERING

# --- Constantes del lazo (no cambian en caliente) ---------------------------
# Periodo FIJO del lazo de control [ms]. La referencia avanza POR TICK, asi que la
# velocidad real es VELOCITY solo si el periodo real del lazo es LOOP_MS. El lazo
# espera lo que falta hasta el proximo tick. Condiciones: dividir a 100 (la
# telemetria sale cada 100 ms) y superar el tiempo de calculo del lazo (la linea
# LOOP del hub informa el periodo medio y los desbordes).
LOOP_MS = 5

# Periodo [ms] con el que se sintonizaron los Ki del PI (lazo libre medido: 2.34 ms).
# Los Ki estan definidos POR TICK a ese periodo: se reescalan por LOOP_MS/KI_REF_MS
# para que el efecto por segundo del PI no dependa del periodo elegido.
KI_REF_MS = 2.34


class Settings:
    """Parametros ajustables desde el dashboard. Una sola instancia: cfg."""

    def __init__(self):
        # Velocidad lineal de referencia [cm/s] (comando VEL).
        self.velocity = v
        # Perfil de velocidad de la referencia (comando PROF): 'TRAP' | 'CONST'.
        self.profile = 'TRAP'
        # Aceleracion/frenado del perfil trapezoidal [cm/s^2] (comando ACC).
        self.accel = 80.0
        # Ganancias de los PI (comando PI). Arrancan con controlConstants.py.
        self.gains = {'kp_pos': KP_POSITION, 'ki_pos': KI_POSITION,
                      'kp_st': KP_STEERING,  'ki_st': KI_STEERING}
        # Cache de la trayectoria calculada al recibir PATH (para graficarla antes
        # del ensayo y no recalcular en START). Se invalida al cambiar PATH.
        self.legs = None
        self.legs_total = 0


cfg = Settings()


# ----------------------------------------------------------------------------
# Parsers de comandos
# ----------------------------------------------------------------------------
def parse_path(arg):
    """'x,y,h;x,y,h;...' -> [[x,y,h], ...]  o None si esta mal formado."""
    poses = []
    for chunk in arg.split(';'):
        chunk = chunk.strip()
        if not chunk:
            continue
        parts = chunk.split(',')
        if len(parts) != 3:
            return None
        try:
            poses.append([float(parts[0]), float(parts[1]), float(parts[2])])
        except:
            return None
    return poses


def set_gains(arg):
    """'kp_pos,ki_pos,kp_st,ki_st' -> actualiza cfg.gains. Devuelve True si ok."""
    parts = arg.split(',')
    if len(parts) != 4:
        return False
    try:
        cfg.gains['kp_pos'] = float(parts[0])
        cfg.gains['ki_pos'] = float(parts[1])
        cfg.gains['kp_st'] = float(parts[2])
        cfg.gains['ki_st'] = float(parts[3])
    except:
        return False
    return True


def set_velocity(arg):
    """'<v>' en cm/s -> actualiza cfg.velocity. Devuelve True si ok (numero > 0)."""
    try:
        val = float(arg)
    except:
        return False
    if val <= 0:
        return False
    cfg.velocity = val
    return True


def set_profile(arg):
    """'CONST' o 'TRAP' -> actualiza cfg.profile. Devuelve True si ok."""
    name = arg.strip().upper()
    if name != 'CONST' and name != 'TRAP':
        return False
    cfg.profile = name
    return True


def set_accel(arg):
    """'<a>' en cm/s^2 -> actualiza cfg.accel. Devuelve True si ok (numero >= 0)."""
    try:
        val = float(arg)
    except:
        return False
    if val < 0:
        return False
    cfg.accel = val
    return True
