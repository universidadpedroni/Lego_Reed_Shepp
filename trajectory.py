# =============================================================================
# trajectory.py - GENERACION DE TRAYECTORIA (planificacion del camino)
# =============================================================================
# A partir de las poses que manda el dashboard (PATH: x, y, rumbo) calcula el camino
# que debe recorrer el auto: una lista de SEGMENTOS por cada par de poses.
#
#   plan_leg(pose_inicial, pose_final)  ->  [{'distance': cm, 'steering': -1/0/+1,
#                                              'gear': +1/-1}, ...]
#
#   steering  -1 curva a la izquierda, 0 recta, +1 curva a la derecha (radio r_turn_min)
#   gear      +1 marcha adelante, -1 marcha atras
#
# El planner es Reeds-Shepp (reeds_shepp.py): camino mas corto con marcha atras permitida.
#
# Tambien arma el camino de TODO el recorrido (build_legs), descarta segmentos de
# distancia casi nula, y manda el camino planificado al dashboard para graficarlo.
# Sin hardware: solo usa print() para hablar con el dashboard.
# =============================================================================
import umath
import gc

from reeds_shepp import get_best_path, normalize_start_and_end_point, denormalize_distance_in_path
from vehicleConstants import r_turn_min
from settings import cfg
from odometry import make_frame

# Reeds-Shepp genera segmentos de transicion de distancia casi nula (los
# giros 't' y 'v' de las curvas CSC suelen dar 0). Si llegaran al lazo de control, la
# referencia no avanzaria y el auto quedaria clavado hasta el watchdog. Se descartan.
MIN_SEG_DISTANCE = 1.0   # [cm]


def plan_leg(start_pose, end_pose):
    """Planifica UN tramo (de una pose a la siguiente) con Reeds-Shepp. Devuelve la lista de segmentos (distancias en cm), o None si no
    hay solucion."""
    sp, ep = normalize_start_and_end_point(start_pose, end_pose, r_turn_min)
    best = get_best_path(sp, ep)
    if best is None:
        return None
    return denormalize_distance_in_path(best, r_turn_min)


def build_legs(PATH):
    """Calcula los tramos (con el filtro de segmentos nulos) de TODO el PATH.
    Devuelve (legs, cantidad_total_de_segmentos) o (None, 0) si algun tramo no
    tiene solucion."""
    legs = []
    seg_total = 0
    for i in range(len(PATH) - 1):
        leg = plan_leg(PATH[i], PATH[i + 1])
        if leg is None:
            return None, 0
        leg = [seg for seg in leg if seg['distance'] >= MIN_SEG_DISTANCE]
        legs.append(leg)
        seg_total += len(leg)
        gc.collect()
    return legs, seg_total


def stream_plan(path, leg_start, step=8.0, batch=14):
    """Integra la trayectoria planificada de un tramo y la manda al dashboard en
    lineas PLAN cortas. No acumula la lista completa de puntos: el pico de memoria
    es de solo "batch" puntos.

    El rumbo se acumula con signo invertido (thl -=) para pasar de la convencion
    'compass' del planner a la matematica estandar usada por cos/sin. Esto solo
    afecta la VISUALIZACION."""
    to_global = make_frame(leg_start)

    xl = 0.0       # posicion local dentro del tramo
    yl = 0.0
    thl = 0.0      # rumbo local (arranca en 0)
    parts = ["{:.1f},{:.1f}".format(leg_start[0], leg_start[1])]

    for seg in path:
        distance = seg['distance']
        steer_dir = seg['steering']
        gear = seg['gear']
        travelled = 0.0
        while travelled < distance:
            ds = step
            if distance - travelled < step:
                ds = distance - travelled
            travelled += ds
            thl -= gear * steer_dir * ds / r_turn_min
            xl += gear * ds * umath.cos(thl)
            yl += gear * ds * umath.sin(thl)
            gx, gy = to_global(xl, yl)
            parts.append("{:.1f},{:.1f}".format(gx, gy))
            if len(parts) >= batch:
                print("PLAN " + ";".join(parts))
                parts = []
    if parts:
        print("PLAN " + ";".join(parts))


def send_plan(legs, PATH):
    """Manda al dashboard la trayectoria planificada de cada tramo (lineas PLAN)."""
    for i in range(len(legs)):
        gc.collect()
        stream_plan(legs[i], PATH[i])
