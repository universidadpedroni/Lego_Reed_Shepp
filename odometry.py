# =============================================================================
# odometry.py - ODOMETRIA (donde esta el auto) y ERRORES RESPECTO DE LA REFERENCIA
# =============================================================================
# Integra la posicion (x, y) del auto a partir de los encoders y el IMU, y calcula
# los errores de traza en el marco de la referencia:
#
#   e_s   atraso (+ adelanto) a lo largo de la traza [cm]
#   e_y   error lateral [cm], + = el auto esta a la IZQUIERDA de la traza
#
# e_y sale de odometria (encoders + IMU): NO es una medida absoluta. Un offset
# inicial real es invisible; si ve todo desvio que aparezca durante el recorrido.
#
# Convencion de signo: el IMU del hub es "compass" (CW positivo). Para integrar con
# cos/sin estandar el rumbo se invierte: heading_math = -heading_imu. Solo afecta a la
# odometria y a la grafica del dashboard; los lazos de control usan el IMU crudo.
# Sin hardware: se prueba en la PC.
# =============================================================================
import umath


class Odometry:
    def __init__(self):
        self.x = 0.0              # posicion estimada en el marco del tramo [cm]
        self.y = 0.0
        self._pos_prev = 0.0      # arco del segmento en el tick anterior [cm]

    def reset_segment(self):
        """Los encoders se ponen a 0 al empezar cada segmento: el arco vuelve a 0."""
        self._pos_prev = 0.0

    def update(self, position_vehicle, heading_deg):
        """Integra un paso. position_vehicle: arco recorrido en el segmento [cm];
        heading_deg: rumbo del IMU [grados, compass]. Devuelve ds [cm] del tick."""
        ds = position_vehicle - self._pos_prev
        self._pos_prev = position_vehicle
        heading_rad = -umath.radians(heading_deg)
        self.x += ds * umath.cos(heading_rad)
        self.y += ds * umath.sin(heading_rad)
        return ds

    def tracking_errors(self, ref):
        """Errores (e_s, e_y) del auto respecto de la referencia `ref` (Reference)."""
        th_r = -ref.heading
        c_r = umath.cos(th_r)
        s_r = umath.sin(th_r)
        dx = self.x - ref.x
        dy = self.y - ref.y
        e_s = c_r * dx + s_r * dy
        e_y = -s_r * dx + c_r * dy
        return e_s, e_y


def make_frame(leg_start):
    """Devuelve to_global(xl, yl): pasa coordenadas locales del tramo (origen en
    leg_start, eje X segun el rumbo inicial) al marco global del dashboard."""
    th0 = umath.radians(leg_start[2])
    c = umath.cos(th0)
    s = umath.sin(th0)
    x0 = leg_start[0]
    y0 = leg_start[1]
    def to_global(xl, yl):
        return (x0 + xl * c - yl * s, y0 + xl * s + yl * c)
    return to_global
