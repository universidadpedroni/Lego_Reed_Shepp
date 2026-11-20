# =============================================================================
# reference.py - REFERENCIA que recorre el camino planificado
# =============================================================================
# Convierte "cuantos cm avanza la referencia en este tick" (lo decide el perfil de
# velocidad, velocity_profile.py) en las tres referencias que persiguen los lazos:
#
#   position  arco recorrido en el tramo [cm]      (lo persigue el PI de traccion)
#   heading   rumbo de referencia [rad, compass]   (lo persigue la direccion)
#   x, y      pose de la referencia en el marco del tramo [cm, convencion matematica]
#             (con ellas se calcula el error lateral e_y y el atraso e_s)
#
# Una Reference se crea por tramo "leg" (de una pose a la siguiente del PATH); dentro
# de un leg encadena sus segmentos (arco/recta) y el rumbo se acumula entre ellos.
# Sin hardware: se prueba en la PC.
# =============================================================================
import umath
from vehicleConstants import r_turn_min


class Reference:
    def __init__(self):
        self.heading = 0.0     # rumbo de referencia acumulado [rad], compass (CW +)
        self.x = 0.0           # pose de la referencia en el marco del tramo
        self.y = 0.0
        self.position = 0.0    # arco de referencia del segmento actual [cm]
        self.done = True
        self._heading0 = 0.0
        self._distance = 0.0
        self._steer_dir = 0
        self._gear = 1
        self._profile = None

    def start_segment(self, distance, steer_dir, gear, profile):
        """Empieza un segmento nuevo. steer_dir: -1 izquierda, 0 recta, +1 derecha
        (convencion del planner); gear: +1 adelante, -1 atras."""
        self._heading0 = self.heading        # rumbo con el que se entra al segmento
        self._distance = distance
        self._steer_dir = steer_dir
        self._gear = gear
        self._profile = profile
        self.position = 0.0
        self.done = False

    def advance(self):
        """Avanza la referencia UN tick (si el segmento no termino)."""
        if self.done:
            return
        gear = self._gear
        ds, last = self._profile.next()
        if not last:
            self.position += gear * ds
            self.heading += gear * self._steer_dir * ds / r_turn_min
        else:
            # Ultimo tick: se clava la referencia en su valor exacto. Cierra justo
            # en 'distance' y no arrastra deriva de redondeo al segmento siguiente.
            self.position = gear * self._distance
            self.heading = self._heading0 + gear * self._steer_dir * self._distance / r_turn_min
            self.done = True
        # Pose de la referencia (convencion matematica, igual que la odometria).
        th = -self.heading
        self.x += gear * ds * umath.cos(th)
        self.y += gear * ds * umath.sin(th)
