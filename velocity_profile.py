# =============================================================================
# velocity_profile.py - PERFILES DE VELOCIDAD (rampas) de la referencia
# =============================================================================
# La referencia NO es el auto: es una "partida" virtual que recorre el camino
# planificado, y el auto la persigue con los lazos de control. Este modulo decide
# CON QUE VELOCIDAD avanza esa referencia, tick a tick.
#
# Cada perfil responde a una sola pregunta: "en este tick, cuantos cm avanzo y
# es el ultimo tick del tramo?" ->  ds, last = perfil.next()
#
# Dos perfiles:
#
#   ConstantProfile   velocidad constante: la referencia arranca y frena de golpe
#                     (un escalon de velocidad). La traccion tiene inercia (tau ~0.15 s)
#                     y no puede seguir un escalon: al frenar el auto se pasa del final.
#
#   TrapezoidProfile  velocidad trapezoidal: sube con aceleracion `accel` hasta la
#                     velocidad maxima, la mantiene, y frena con la misma rampa para
#                     llegar al final con velocidad v1:
#
#        v ^          ____________
#          |         /            \      <- area bajo la curva = distancia del tramo
#          |        /              \
#          |_______/                \____> t
#                 rampa      rampa
#                 de subida  de frenado
#
#   La rampa de frenado se calcula con la distancia que FALTA (cinematica):
#       v_max_permitida = sqrt(v1^2 + 2 * accel * distancia_que_falta)
#   asi, aunque el tramo sea corto para llegar a la velocidad maxima, el perfil se
#   vuelve triangular solo.
#
# make_profile() elige cual usar y fija las velocidades de borde del tramo (v0, v1):
# la referencia arranca/termina en reposo SOLO al inicio del recorrido, al final y en
# los cambios de marcha (cusps). Entre tramos del mismo sentido sigue a velocidad
# maxima: el auto no se detiene en el medio de una curva.
#
# No usa hardware ni otros modulos del proyecto: se prueba entero en la PC.
# =============================================================================
import umath

V_MIN_REF = 2.0   # [cm/s] piso de la rampa de frenado: si no, la referencia
                  # se acercaria al final cada vez mas despacio sin llegar nunca.


class ConstantProfile:
    """Velocidad constante. Cuantiza el tramo a N ticks enteros y recalcula el paso
    ds = distancia/N para caer EXACTO en `distance` (criterio Euler de paso fijo)."""

    def __init__(self, distance, v_max, h_ms):
        step = h_ms * v_max / 1000.0               # cm por tick a velocidad maxima
        n = int(distance / step)
        if n * step < distance:
            n += 1
        if n < 1:
            n = 1
        self.n = n
        self.ds = distance / n
        self.ticks = 0

    def next(self):
        """Devuelve (ds [cm], last [bool])."""
        self.ticks += 1
        return self.ds, self.ticks >= self.n


class TrapezoidProfile:
    """Trapecio de velocidad: acelera, crucero, frena. v0 / v1: velocidad [cm/s] al
    empezar / terminar el tramo (0 = reposo)."""

    def __init__(self, distance, v0, v1, v_max, accel, h_ms):
        self.distance = distance
        self.v1 = v1
        self.v_max = v_max
        self.accel = accel
        self.h_ms = h_ms
        self.v = v0          # velocidad actual de la referencia [cm/s]
        self.s = 0.0         # distancia ya recorrida por la referencia [cm]

    def next(self):
        """Devuelve (ds [cm], last [bool])."""
        remaining = self.distance - self.s
        v_up = self.v + self.accel * self.h_ms / 1000.0                          # rampa de subida
        v_brake = umath.sqrt(self.v1 * self.v1 + 2.0 * self.accel * remaining)   # rampa de frenado
        v = min(self.v_max, v_up, v_brake)
        if v < V_MIN_REF:
            v = V_MIN_REF
        self.v = v
        ds = v * self.h_ms / 1000.0
        last = ds >= remaining
        if last:
            ds = remaining
        self.s += ds
        return ds, last


def make_profile(kind, distance, gear, prev_gear, next_gear, v_max, accel, h_ms):
    """Arma el perfil de un tramo.
      kind       'TRAP' o 'CONST'
      gear       sentido de este tramo (+1 adelante, -1 marcha atras)
      prev_gear  sentido del tramo anterior (None si es el primero del recorrido)
      next_gear  sentido del tramo siguiente (None si es el ultimo)
    Con kind='CONST' (o accel = 0) devuelve velocidad constante."""
    if kind == 'TRAP' and accel > 0:
        v0 = v_max if prev_gear == gear else 0.0     # None != gear -> arranca de reposo
        v1 = v_max if next_gear == gear else 0.0     # None != gear -> termina en reposo
        return TrapezoidProfile(distance, v0, v1, v_max, accel, h_ms)
    return ConstantProfile(distance, v_max, h_ms)
