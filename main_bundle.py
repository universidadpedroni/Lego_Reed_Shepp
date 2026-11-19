# ============================================================================
# main_bundle.py - GENERADO por SMC/make_bundle.py (2026-11-19 21:07). NO EDITAR A MANO:
# editar los .py originales y volver a correr el script.
# ============================================================================
import umath
import gc
from pybricks.hubs import TechnicHub
from pybricks.parameters import Color, Direction, Port
from pybricks.tools import wait, StopWatch
from pybricks.robotics import Car
from pybricks.pupdevices import Motor
from usys import stdin
from uselect import poll


# ============================================================================
# ---- modulo: vehicleConstants.py
# ============================================================================
L_car = 24.0 #[cm] Distancia ente ejes
psi_max = 20.0 #[°] Máximo radio de giro
v = 40.0 #[cm/seg] Velocidad lineal constante del vehículo
#r_turn_min = 70.1713    # L_car/sin(psi_max), radio mínimo de giro
r_turn_min = 80.0        # Hacemos el radio de giro mínimo un poco más grande para darle espacio al control de dirección

max_drive_power = 100
max_angle_power = 100
    


# ============================================================================
# ---- modulo: controlConstants.py
# ============================================================================
# controlConstants.py

# Ganancias de control Proporcional-Integral (PI).
# Son la semilla inicial de GAINS en main_rs.py. El dashboard pisa estos
# valores con el comando PID en cada START, asi que solo se usan al bootear
# el hub o si se corre el firmware sin dashboard.
KP_POSITION = 50.0
KI_POSITION = 0.01

KP_STEERING = 50.0
KI_STEERING = 0.01


# ============================================================================
# ---- modulo: reeds_shepp.py
# ============================================================================
#import math as umath

FORWARD = 1
BACKWARD = -1
LEFT = -1
RIGHT = 1
STRAIGHT = 0

def denormalize_distance_in_path(path, r_turn_min):
    for p in path:
        p['distance'] = p['distance'] * r_turn_min  # Escalar la distancia real
    return path

def normalize_start_and_end_point(start_point_original, end_point_original, r_turn_min):
    # Inicializar el array de puntos finales
    start_point = [0, 0, 0]
    end_point = [0, 0, 0]

    # Dividir el valor por r_turn_min (definir 'r_turn_min' previamente)
    start_point[0] = start_point_original[0] / r_turn_min
    start_point[1] = start_point_original[1] / r_turn_min
    start_point[2] = umath.radians(start_point_original[2]) 
    
    end_point[0] = end_point_original[0] / r_turn_min
    end_point[1] = end_point_original[1] / r_turn_min
    end_point[2] =  umath.radians(end_point_original[2]) 
    return start_point, end_point

def R(x, y):
    """ Calcula u y t usando la función R(x, y) de Reeds-Shepp """
    u = umath.sqrt(x**2 + y**2)  # Hipotenusa
    t = umath.atan2(y, x)  # Ángulo
    return u, t

def M(theta):
    """
    Devuelve el ángulo phi = theta mod (2*pi) en el rango -pi <= phi < pi.
    """
    theta = umath.fmod(theta, 2 * umath.pi)  # math.fmod mantiene el signo
    if theta < -umath.pi:
        theta += 2 * umath.pi
    elif theta >= umath.pi:
        theta -= 2 * umath.pi
    return theta

def create_path_element(distance, steering, gear):
    """ Crea un diccionario representando un segmento del camino """
    if distance < 0:
        gear = -gear
    return {"distance": abs(distance), "steering": steering, "gear": gear}


def change_of_basis(start_point, end_point):
    """ Calcula la coordenada de 'end' en el sistema donde 'start' es (0,0,0) """
    x, y, theta = start_point
    x_end, y_end, theta_end = end_point

    dx = x_end - x
    dy = y_end - y
    dtheta = M(theta_end - theta)

    x = dx * umath.cos(theta) + dy * umath.sin(theta)
    y = -dx * umath.sin(theta) + dy * umath.cos(theta)

    return x, y, dtheta

def path1(x, y, phi):
    """ Calcula la trayectoria CSC (same turns) """
    path = []

    u, t = R(x - umath.sin(phi), y - 1 + umath.cos(phi))
    v = M(phi - t)

    path.append(create_path_element(t, LEFT, FORWARD))
    path.append(create_path_element(u, STRAIGHT, FORWARD))
    path.append(create_path_element(v, LEFT, FORWARD))

    return path

def path2(x, y, phi):
    """ Calcula la trayectoria CSC (opposite turns) """
    phi = M(phi)
    path = []

    rho, t1 = R(x + umath.sin(phi), y - 1 - umath.cos(phi))

    if rho * rho >= 4:
        u = umath.sqrt(rho * rho - 4)
        t = M(t1 + umath.atan2(2, u))
        v = M(t - phi)

        path.append(create_path_element(t, LEFT, FORWARD))
        path.append(create_path_element(u, STRAIGHT, FORWARD))
        path.append(create_path_element(v, RIGHT, FORWARD))

    return path

def path3(x, y, phi):
    """ Calcula la trayectoria C|C|C """
    path = []

    xi = x - umath.sin(phi)
    eta = y - 1 + umath.cos(phi)
    rho, theta = R(xi, eta)

    if rho <= 4:
        A = umath.acos(rho / 4)
        t = M(theta + umath.pi / 2 + A)
        u = M(umath.pi - 2 * A)
        v = M(phi - t - u)

        path.append(create_path_element(t, LEFT, FORWARD))
        path.append(create_path_element(u, RIGHT, BACKWARD))
        path.append(create_path_element(v, LEFT, FORWARD))

    return path

def path4(x, y, phi):
    """ Calcula la trayectoria C|CC """
    path = []

    xi = x - umath.sin(phi)
    eta = y - 1 + umath.cos(phi)
    rho, theta = R(xi, eta)

    if rho <= 4:
        A = umath.acos(rho / 4)
        t = M(theta + umath.pi / 2 + A)
        u = M(umath.pi - 2 * A)
        v = M(t + u - phi)

        path.append(create_path_element(t, LEFT, FORWARD))
        path.append(create_path_element(u, RIGHT, BACKWARD))
        path.append(create_path_element(v, LEFT, BACKWARD))

    return path

def path5(x, y, phi):
    """ Calcula la trayectoria CC|C """
    path = []

    xi = x - umath.sin(phi)
    eta = y - 1 + umath.cos(phi)
    rho, theta = R(xi, eta)

    if rho <= 4:
        u = umath.acos(1 - (rho ** 2) / 8)
        A = umath.asin(2 * umath.sin(u) / rho)
        t = M(theta + umath.pi / 2 - A)
        v = M(t - u - phi)

        path.append(create_path_element(t, LEFT, FORWARD))
        path.append(create_path_element(u, RIGHT, FORWARD))
        path.append(create_path_element(v, LEFT, BACKWARD))

    return path

def path6(x, y, phi):
    """ Calcula la trayectoria CCu|CuC """
    path = []

    xi = x + umath.sin(phi)
    eta = y - 1 - umath.cos(phi)
    rho, theta = R(xi, eta)

    if rho <= 4:
        if rho <= 2:
            A = umath.acos((rho + 2) / 4)
            t = M(theta + umath.pi / 2 + A)
            u = M(A)
            v = M(phi - t + 2 * u)
        else:
            A = umath.acos((rho - 2) / 4)
            t = M(theta + umath.pi / 2 - A)
            u = M(umath.pi - A)
            v = M(phi - t + 2 * u)

        path.append(create_path_element(t, LEFT, FORWARD))
        path.append(create_path_element(u, RIGHT, FORWARD))
        path.append(create_path_element(u, LEFT, BACKWARD))
        path.append(create_path_element(v, RIGHT, BACKWARD))

    return path

def path7(x, y, phi):
    """ Calcula la trayectoria C|CuCu|C """
    path = []

    xi = x + umath.sin(phi)
    eta = y - 1 - umath.cos(phi)
    rho, theta = R(xi, eta)
    u1 = (20 - rho * rho) / 16

    if rho <= 6 and 0 <= u1 <= 1:
        u = umath.acos(u1)
        A = umath.asin(2 * umath.sin(u) / rho)
        t = M(theta + umath.pi / 2 + A)
        v = M(t - phi)

        path.append(create_path_element(t, LEFT, FORWARD))
        path.append(create_path_element(u, RIGHT, BACKWARD))
        path.append(create_path_element(u, LEFT, BACKWARD))
        path.append(create_path_element(v, RIGHT, FORWARD))

    return path

def path8(x, y, phi):
    """ Calcula la trayectoria C|C[pi/2]SC """
    path = []

    xi = x - umath.sin(phi)
    eta = y - 1 + umath.cos(phi)
    rho, theta = R(xi, eta)

    if rho >= 2:
        u = umath.sqrt(rho * rho - 4) - 2
        A = umath.atan2(2, u + 2)
        t = M(theta + umath.pi / 2 + A)
        v = M(t - phi + umath.pi / 2)

        path.append(create_path_element(t, LEFT, FORWARD))
        path.append(create_path_element(umath.pi / 2, RIGHT, BACKWARD))
        path.append(create_path_element(u, STRAIGHT, BACKWARD))
        path.append(create_path_element(v, LEFT, BACKWARD))

    return path

def path9(x, y, phi):
    """ Calcula la trayectoria CSC[pi/2]|C """
    path = []

    xi = x - umath.sin(phi)
    eta = y - 1 + umath.cos(phi)
    rho, theta = R(xi, eta)

    if rho >= 2:
        u = umath.sqrt(rho * rho - 4) - 2
        A = umath.atan2(u + 2, 2)
        t = M(theta + umath.pi / 2 - A)
        v = M(t - phi - umath.pi / 2)

        path.append(create_path_element(t, LEFT, FORWARD))
        path.append(create_path_element(u, STRAIGHT, FORWARD))
        path.append(create_path_element(umath.pi / 2, RIGHT, FORWARD))
        path.append(create_path_element(v, LEFT, BACKWARD))

    return path

def path10(x, y, phi):
    """ Calcula la trayectoria C|C[pi/2]SC """
    path = []

    xi = x + umath.sin(phi)
    eta = y - 1 - umath.cos(phi)
    rho, theta = R(xi, eta)

    if rho >= 2:
        t = M(theta + umath.pi / 2)
        u = rho - 2
        v = M(phi - t - umath.pi / 2)

        path.append(create_path_element(t, LEFT, FORWARD))
        path.append(create_path_element(umath.pi / 2, RIGHT, BACKWARD))
        path.append(create_path_element(u, STRAIGHT, BACKWARD))
        path.append(create_path_element(v, RIGHT, BACKWARD))

    return path

def path11(x, y, phi):
    """ Calcula la trayectoria CSC[pi/2]|C """
    path = []

    xi = x + umath.sin(phi)
    eta = y - 1 - umath.cos(phi)
    rho, theta = R(xi, eta)

    if rho >= 2:
        t = M(theta)
        u = rho - 2
        v = M(phi - t - umath.pi / 2)

        path.append(create_path_element(t, LEFT, FORWARD))
        path.append(create_path_element(u, STRAIGHT, FORWARD))
        path.append(create_path_element(umath.pi / 2, LEFT, FORWARD))
        path.append(create_path_element(v, RIGHT, BACKWARD))

    return path

def path12(x, y, phi):
    """ Calcula la trayectoria C|C[pi/2]SC[pi/2]|C """
    path = []

    xi = x + umath.sin(phi)
    eta = y - 1 - umath.cos(phi)
    rho, theta = R(xi, eta)

    if rho >= 4:
        u = umath.sqrt(rho * rho - 4) - 4
        A = umath.atan2(2, u + 4)
        t = M(theta + umath.pi / 2 + A)
        v = M(t - phi)

        path.append(create_path_element(t, LEFT, FORWARD))
        path.append(create_path_element(umath.pi / 2, RIGHT, BACKWARD))
        path.append(create_path_element(u, STRAIGHT, BACKWARD))
        path.append(create_path_element(umath.pi / 2, LEFT, BACKWARD))
        path.append(create_path_element(v, RIGHT, FORWARD))

    return path

def reverse_gear(element):
    """Invierte la dirección de la marcha."""
    element["gear"] *= -1
    return element

def reverse_steering(element):
    """Invierte el sentido de giro."""
    element["steering"] *= -1
    return element

def reflect(path):
    """Refleja el camino invirtiendo el sentido de giro de cada elemento."""
    new_path = []
    for element in path:
        new_path.append(reverse_steering(element.copy()))  # Copia para evitar modificar el original
    return new_path

def timeflip(path):
    """Realiza la transformación timeflip invirtiendo el sentido de marcha de cada elemento."""
    new_path = []
    for element in path:
        new_path.append(reverse_gear(element.copy()))  # Copia para evitar modificar el original
    return new_path


def get_all_paths(start_point, end_point):
    """
    Retorna una lista con todos los caminos generados por las 12 funciones y sus variantes.
    """

    # Lista de funciones de trayectoria
    path_fns = [path1, path2, path3, path4, path5, path6,
                path7, path8, path9, path10, path11, path12]

    paths = []

    # Obtener las coordenadas de 'end' en el sistema donde 'start' es (0,0,0)
    x, y, theta = change_of_basis(start_point, end_point)

    for get_path in path_fns:
        # Obtener las cuatro variantes de cada tipo de path
        var1 = get_path(x, y, theta)  # Variante original
        var2 = timeflip(get_path(-x, y, -theta))  # Variante con timeflip
        var3 = reflect(get_path(x, -y, -theta))  # Variante con reflect
        var4 = reflect(timeflip(get_path(-x, -y, theta)))  # Variante con timeflip y reflect

        # Agregar las trayectorias no vacías a la lista principal
        if var1:
            paths.append(var1)
        if var2:
            paths.append(var2)
        if var3:
            paths.append(var3)
        if var4:
            paths.append(var4)

    return paths

def get_optimal_path(paths):
    min_distance = float('inf')
    min_distance_index = -1
    
    for i, path in enumerate(paths):
        # Inicializamos la distancia acumulada
        path_distance = 0
        
        # Sumamos las distancias de cada elemento en `path`
        for p in path:
            
            path_distance += p['distance']
        
        # Comparamos la distancia total con la mínima
        if path_distance < min_distance:
            min_distance = path_distance
            min_distance_index = i
    
    return min_distance_index

def get_best_path(start_point, end_point):
    """Camino Reeds-Shepp mas corto. Evalua las variantes de a una y se
    queda solo con la mejor, sin materializar las 48 a la vez: la RAM del
    Technic Hub no alcanza. Devuelve el path o None si no hay solucion."""
    path_fns = [path1, path2, path3, path4, path5, path6,
                path7, path8, path9, path10, path11, path12]

    x, y, theta = change_of_basis(start_point, end_point)

    best = None
    best_len = 0.0

    for get_path in path_fns:
        variants = (get_path(x, y, theta),
                    timeflip(get_path(-x, y, -theta)),
                    reflect(get_path(x, -y, -theta)),
                    reflect(timeflip(get_path(-x, -y, theta))))
        for v in variants:
            if not v:
                continue
            d = 0.0
            for p in v:
                d += p['distance']
            if best is None or d < best_len:
                best_len = d
                best = v
        gc.collect()

    return best


# ============================================================================
# ---- modulo: dubinsPathPlanning.py
# ============================================================================
# dubinsPathPlanning.py
#
# Planificador de Dubins (solo marcha adelante). Mismo estilo que
# reeds_shepp.py: cada candidato (RSR, LSL, LSR, RSL, RLR, LRL) se evalua
# de a uno y solo se conserva el mejor. Asi evitamos materializar las 6
# trayectorias a la vez (poca RAM en el Technic Hub).
#
# La salida es la misma lista de dicts que produce reeds_shepp.get_best_path
# despues de denormalizar:  [{'distance', 'steering', 'gear'}, ...].
# El campo 'gear' es siempre FORWARD: Dubins no admite marcha atras.


# Convencion compartida con reeds_shepp para que el controlador del auto
# trate ambos planners por igual.


# ----------------------------------------------------------------------------
# Helpers
# ----------------------------------------------------------------------------
def _arc_signed(theta_arc, turn_dir):
    """Normaliza el angulo de un arco al rango correcto segun el sentido de
    giro. 'L' espera arco positivo (CCW desde el centro), 'R' negativo."""
    if turn_dir == 'L' and theta_arc < 0.0:
        theta_arc += 2.0 * umath.pi
    elif turn_dir == 'R' and theta_arc > 0.0:
        theta_arc -= 2.0 * umath.pi
    return theta_arc


def _tangents_CSC(c1, c2, r, path_type):
    """Puntos tangentes para un trayecto C-S-C entre dos circulos de radio
    r centrados en c1 y c2. path_type en {'RSR','LSL','RSL','LSR'}.
    Devuelve [x1, y1, x2, y2] o [] si no hay solucion."""
    x1, y1 = c1
    x2, y2 = c2

    dx = x2 - x1
    dy = y2 - y1
    d_sq = dx * dx + dy * dy
    if d_sq < 1e-12:
        return []

    d = umath.sqrt(d_sq)
    vx = dx / d
    vy = dy / d

    tangents = []
    for sign1 in (+1, -1):
        c = (r - sign1 * r) / d
        if c * c > 1:
            continue
        h = umath.sqrt(max(0.0, 1 - c * c))
        for sign2 in (+1, -1):
            nx = vx * c - sign2 * h * vy
            ny = vy * c + sign2 * h * vx
            tangents.append([
                x1 + r * nx,
                y1 + r * ny,
                x2 + sign1 * r * nx,
                y2 + sign1 * r * ny,
            ])

    idx = {'RSR': 0, 'LSL': 1, 'RSL': 2, 'LSR': 3}.get(path_type, -1)
    if idx < 0 or idx >= len(tangents):
        return []
    return tangents[idx]


# ----------------------------------------------------------------------------
# Variantes Dubins. Cada una devuelve (L, path_type) donde L es la lista
# [arco1, recta_o_arco, arco_o_arco] en cm. Devuelve (None, path_type) si
# la geometria no admite esa variante.
# ----------------------------------------------------------------------------
def dubin_RSR(s, g, r):
    pt = 'RSR'
    c1 = (s[0] + r * umath.cos(s[2] - umath.pi / 2),
          s[1] + r * umath.sin(s[2] - umath.pi / 2))
    c2 = (g[0] + r * umath.cos(g[2] - umath.pi / 2),
          g[1] + r * umath.sin(g[2] - umath.pi / 2))
    D = umath.sqrt((c2[0] - c1[0]) ** 2 + (c2[1] - c1[1]) ** 2)
    if D < 2 * r:
        return None, pt

    t = _tangents_CSC(c1, c2, r, pt)
    if not t:
        return None, pt

    a1 = _arc_signed(umath.atan2(t[1] - c1[1], t[0] - c1[0])
                     - umath.atan2(s[1] - c1[1], s[0] - c1[0]), 'R')
    L0 = r * abs(a1)
    L1 = umath.sqrt((t[2] - t[0]) ** 2 + (t[3] - t[1]) ** 2)
    a3 = _arc_signed(umath.atan2(g[1] - c2[1], g[0] - c2[0])
                     - umath.atan2(t[3] - c2[1], t[2] - c2[0]), 'R')
    L2 = r * abs(a3)
    return [L0, L1, L2], pt


def dubin_LSL(s, g, r):
    pt = 'LSL'
    c1 = (s[0] + r * umath.cos(s[2] + umath.pi / 2),
          s[1] + r * umath.sin(s[2] + umath.pi / 2))
    c2 = (g[0] + r * umath.cos(g[2] + umath.pi / 2),
          g[1] + r * umath.sin(g[2] + umath.pi / 2))
    D = umath.sqrt((c2[0] - c1[0]) ** 2 + (c2[1] - c1[1]) ** 2)
    if D < 2 * r:
        return None, pt

    t = _tangents_CSC(c1, c2, r, pt)
    if not t:
        return None, pt

    a1 = _arc_signed(umath.atan2(t[1] - c1[1], t[0] - c1[0])
                     - umath.atan2(s[1] - c1[1], s[0] - c1[0]), 'L')
    L0 = r * abs(a1)
    L1 = umath.sqrt((t[2] - t[0]) ** 2 + (t[3] - t[1]) ** 2)
    a3 = _arc_signed(umath.atan2(g[1] - c2[1], g[0] - c2[0])
                     - umath.atan2(t[3] - c2[1], t[2] - c2[0]), 'L')
    L2 = r * abs(a3)
    return [L0, L1, L2], pt


def dubin_LSR(s, g, r):
    pt = 'LSR'
    c1 = (s[0] + r * umath.cos(s[2] + umath.pi / 2),
          s[1] + r * umath.sin(s[2] + umath.pi / 2))
    c2 = (g[0] + r * umath.cos(g[2] - umath.pi / 2),
          g[1] + r * umath.sin(g[2] - umath.pi / 2))
    D = umath.sqrt((c2[0] - c1[0]) ** 2 + (c2[1] - c1[1]) ** 2)
    if D < 2 * r:
        return None, pt

    t = _tangents_CSC(c1, c2, r, pt)
    if not t:
        return None, pt

    a1 = _arc_signed(umath.atan2(t[1] - c1[1], t[0] - c1[0])
                     - umath.atan2(s[1] - c1[1], s[0] - c1[0]), 'L')
    L0 = r * abs(a1)
    L1 = umath.sqrt((t[2] - t[0]) ** 2 + (t[3] - t[1]) ** 2)
    a3 = _arc_signed(umath.atan2(g[1] - c2[1], g[0] - c2[0])
                     - umath.atan2(t[3] - c2[1], t[2] - c2[0]), 'R')
    L2 = r * abs(a3)
    return [L0, L1, L2], pt


def dubin_RSL(s, g, r):
    pt = 'RSL'
    c1 = (s[0] + r * umath.cos(s[2] - umath.pi / 2),
          s[1] + r * umath.sin(s[2] - umath.pi / 2))
    c2 = (g[0] + r * umath.cos(g[2] + umath.pi / 2),
          g[1] + r * umath.sin(g[2] + umath.pi / 2))
    D = umath.sqrt((c2[0] - c1[0]) ** 2 + (c2[1] - c1[1]) ** 2)
    if D < 2 * r:
        return None, pt

    t = _tangents_CSC(c1, c2, r, pt)
    if not t:
        return None, pt

    a1 = _arc_signed(umath.atan2(t[1] - c1[1], t[0] - c1[0])
                     - umath.atan2(s[1] - c1[1], s[0] - c1[0]), 'R')
    L0 = r * abs(a1)
    L1 = umath.sqrt((t[2] - t[0]) ** 2 + (t[3] - t[1]) ** 2)
    a3 = _arc_signed(umath.atan2(g[1] - c2[1], g[0] - c2[0])
                     - umath.atan2(t[3] - c2[1], t[2] - c2[0]), 'L')
    L2 = r * abs(a3)
    return [L0, L1, L2], pt


def dubin_RLR(s, g, r):
    pt = 'RLR'
    c1 = (s[0] + r * umath.cos(s[2] - umath.pi / 2),
          s[1] + r * umath.sin(s[2] - umath.pi / 2))
    c2 = (g[0] + r * umath.cos(g[2] - umath.pi / 2),
          g[1] + r * umath.sin(g[2] - umath.pi / 2))
    D = umath.sqrt((c2[0] - c1[0]) ** 2 + (c2[1] - c1[1]) ** 2)
    if D > 4 * r * 0.95:
        return None, pt

    alpha = umath.atan2(c2[1] - c1[1], c2[0] - c1[0])
    beta = umath.acos(D / (4 * r))
    theta = alpha - beta
    c3 = (c1[0] + 2 * r * umath.cos(theta),
          c1[1] + 2 * r * umath.sin(theta))

    norm1 = umath.sqrt((c1[0] - c3[0]) ** 2 + (c1[1] - c3[1]) ** 2)
    norm2 = umath.sqrt((c2[0] - c3[0]) ** 2 + (c2[1] - c3[1]) ** 2)
    pt1 = (c3[0] + r * (c1[0] - c3[0]) / norm1,
           c3[1] + r * (c1[1] - c3[1]) / norm1)
    pt2 = (c3[0] + r * (c2[0] - c3[0]) / norm2,
           c3[1] + r * (c2[1] - c3[1]) / norm2)

    a1 = _arc_signed(umath.atan2(pt1[1] - c1[1], pt1[0] - c1[0])
                     - umath.atan2(s[1] - c1[1], s[0] - c1[0]), 'R')
    a2 = _arc_signed(umath.atan2(pt2[1] - c3[1], pt2[0] - c3[0])
                     - umath.atan2(pt1[1] - c3[1], pt1[0] - c3[0]), 'L')
    a3 = _arc_signed(umath.atan2(g[1] - c2[1], g[0] - c2[0])
                     - umath.atan2(pt2[1] - c2[1], pt2[0] - c2[0]), 'R')
    return [r * abs(a1), r * abs(a2), r * abs(a3)], pt


def dubin_LRL(s, g, r):
    pt = 'LRL'
    c1 = (s[0] + r * umath.cos(s[2] + umath.pi / 2),
          s[1] + r * umath.sin(s[2] + umath.pi / 2))
    c2 = (g[0] + r * umath.cos(g[2] + umath.pi / 2),
          g[1] + r * umath.sin(g[2] + umath.pi / 2))
    D = umath.sqrt((c2[0] - c1[0]) ** 2 + (c2[1] - c1[1]) ** 2)
    if D > 4 * r * 0.95:
        return None, pt

    alpha = umath.atan2(c2[1] - c1[1], c2[0] - c1[0])
    beta = umath.acos(D / (4 * r))
    theta = alpha + beta
    c3 = (c1[0] + 2 * r * umath.cos(theta),
          c1[1] + 2 * r * umath.sin(theta))

    norm1 = umath.sqrt((c1[0] - c3[0]) ** 2 + (c1[1] - c3[1]) ** 2)
    norm2 = umath.sqrt((c2[0] - c3[0]) ** 2 + (c2[1] - c3[1]) ** 2)
    pt1 = (c3[0] + r * (c1[0] - c3[0]) / norm1,
           c3[1] + r * (c1[1] - c3[1]) / norm1)
    pt2 = (c3[0] + r * (c2[0] - c3[0]) / norm2,
           c3[1] + r * (c2[1] - c3[1]) / norm2)

    a1 = _arc_signed(umath.atan2(pt1[1] - c1[1], pt1[0] - c1[0])
                     - umath.atan2(s[1] - c1[1], s[0] - c1[0]), 'L')
    a2 = _arc_signed(umath.atan2(pt2[1] - c3[1], pt2[0] - c3[0])
                     - umath.atan2(pt1[1] - c3[1], pt1[0] - c3[0]), 'R')
    a3 = _arc_signed(umath.atan2(g[1] - c2[1], g[0] - c2[0])
                     - umath.atan2(pt2[1] - c2[1], pt2[0] - c2[0]), 'L')
    return [r * abs(a1), r * abs(a2), r * abs(a3)], pt


# ----------------------------------------------------------------------------
# Seleccion del mejor candidato. Estilo lazy: evalua de a uno, conserva el
# mejor, libera el resto. gc.collect() entre iteraciones igual que en
# reeds_shepp.get_best_path.
# ----------------------------------------------------------------------------
def get_best_dubins(start_pose, end_pose, r_turn_min):
    """Camino Dubins mas corto entre start_pose y end_pose. Las poses entran
    como (x_cm, y_cm, h_grados). Devuelve la lista en el formato unificado
    [{distance, steering, gear}, ...] o None si ninguna variante es valida."""
    s = (start_pose[0], start_pose[1], umath.radians(start_pose[2]))
    g = (end_pose[0],   end_pose[1],   umath.radians(end_pose[2]))

    variant_fns = (dubin_RSR, dubin_LSL, dubin_LSR, dubin_RSL,
                   dubin_RLR, dubin_LRL)
    steer_map = {'L': LEFT, 'R': RIGHT, 'S': STRAIGHT}

    best = None
    best_len = 0.0

    for fn in variant_fns:
        L, path_type = fn(s, g, r_turn_min)
        if L is None:
            gc.collect()
            continue
        d_total = L[0] + L[1] + L[2]
        if best is None or d_total < best_len:
            best_len = d_total
            best = [{"distance": L[i],
                     "steering": steer_map[path_type[i]],
                     "gear": FORWARD} for i in range(3)]
        gc.collect()

    return best


# ----------------------------------------------------------------------------
# Test standalone
# ----------------------------------------------------------------------------


# ============================================================================
# ---- modulo: smc_control.py
# ============================================================================
# =============================================================================
# smc_control.py - Controladores por MODOS DESLIZANTES (SMC) para el AUTO LEGO
# =============================================================================
# Modulo puro (solo math): corre igual en MicroPython (umath) y en CPython, asi
# el mismo codigo que se simula en sim_smc.py es el que va al hub.
#
# CONVENCION (matematica, la de la odometria del dashboard):
#   - rumbo theta: CCW positivo (= -heading del IMU, que es "compass" CW+)
#   - kappa (curvatura) > 0: gira a la izquierda
#   - e_y > 0: el auto esta a la izquierda de la referencia
#   - gear = +1 adelante, -1 marcha atras
# El signo del comando de direccion que sale de aca es matematico (+ = izquierda).
# El firmware lo manda con steer(-pct) porque Pybricks/IMU usan compass CW+.
#
# Referencia teorica y deduccion completa: SMC/SMC_diseno.md
# =============================================================================
try:
    import umath as _m
except ImportError:
    import math as _m


def sat(x):
    """Saturacion unitaria: sign(x) suavizado en la capa limite |x|<1."""
    if x > 1.0:
        return 1.0
    if x < -1.0:
        return -1.0
    return x


def wrap_pi(a):
    """Lleva un angulo a [-pi, pi)."""
    while a >= _m.pi:
        a -= 2.0 * _m.pi
    while a < -_m.pi:
        a += 2.0 * _m.pi
    return a


class LateralSMC:
    """SMC de DIRECCION: regula juntos el error lateral e_y y el de rumbo e_theta.

    Rumbo geometrico: e_g = e_theta - kappa_ref*e_s   (rumbo del camino en el punto
                      mas cercano; sin esto el atraso a lo largo de la traza e_s
                      sesga el error lateral en las curvas: e_y ~ kappa*e_s/lam)
    Superficie:      s = e_g + lam * gear * e_y
    Equivalente:     kappa_eq  = kappa_ref*cos(e_g)/(1 - kappa_ref*e_y) - gear*lam*sin(e_g)
    Control:         kappa_cmd = kappa_eq - gear * (eta*sat(s/phi) + ki*z)
    Direccion:       delta = atan(L*kappa_cmd)  ->  pct = 100*delta/delta_max

    Con eta > (rho*|kappa_eq|max + d)/(1-rho) se cumple s*ds/dt <= -|v|*eta*(1-rho)*|s|
    fuera de la capa limite (rho = error relativo de ganancia de direccion,
    d = sesgo de curvatura). Sobre s=0: de_y/dt = -|v|*lam*e_y (converge sin
    importar el sentido de marcha porque la superficie lleva el factor 'gear').
    """

    def __init__(self, L, delta_max_deg, lam, eta, phi, ki=0.0):
        self.L = L
        self.delta_max = _m.radians(delta_max_deg)
        self.lam = lam      # [1/cm]  polo de e_y sobre la superficie: |v|*lam [1/s]
        self.eta = eta      # [1/cm]  ganancia conmutada (en curvatura)
        self.phi = phi      # [rad]   espesor de la capa limite
        self.ki = ki        # [1/(cm*rad*cm)] integral opcional dentro de la capa
        self.z = 0.0
        self.s = 0.0
        self.pct_eq = 0.0    # % de direccion aportado por el control equivalente
        self.pct_sw = 0.0    # % aportado por el termino conmutado (+ integral)

    def reset(self):
        """Llamar al arrancar cada segmento (el signo de la superficie cambia
        con el gear y el integrador no debe arrastrarse)."""
        self.z = 0.0
        self.s = 0.0
        self.pct_eq = 0.0
        self.pct_sw = 0.0

    def update(self, gear, e_theta, e_y, e_s, kappa_ref, ds_abs, kappa_ff=None):
        """gear: +1/-1. e_theta [rad], e_y y e_s [cm] (atraso/adelanto a lo largo de la traza), kappa_ref [1/cm] firmado,
        ds_abs [cm] = arco recorrido en este tick (|v|*dt), para el integrador,
        kappa_ff [1/cm] = curvatura anticipada para el feedforward (None = kappa_ref).
        Devuelve el comando de direccion en % (+ = izquierda, saturado a +-100)."""
        g = 1.0 if gear >= 0 else -1.0
        e_g = wrap_pi(e_theta - kappa_ref * e_s)
        s = e_g + self.lam * g * e_y
        if kappa_ff is None:
            kappa_ff = kappa_ref      # anticipacion opcional (preview) de la curvatura
        k_eq = kappa_ff * _m.cos(e_g) / (1.0 - kappa_ref * e_y) - g * self.lam * _m.sin(e_g)
        sw = self.eta * sat(s / self.phi)
        if self.ki > 0.0 and abs(s) < self.phi:        # integra solo dentro de la capa
            self.z += s * ds_abs
            z_max = 0.5 * self.eta / self.ki
            if self.z > z_max:
                self.z = z_max
            elif self.z < -z_max:
                self.z = -z_max
        k_cmd = k_eq - g * (sw + self.ki * self.z)
        delta = _m.atan(self.L * k_cmd)
        pct = 100.0 * delta / self.delta_max
        if pct > 100.0:
            pct = 100.0
        elif pct < -100.0:
            pct = -100.0
        self.s = s
        self.pct_eq = 100.0 * _m.atan(self.L * k_eq) / self.delta_max
        self.pct_sw = pct - self.pct_eq          # eq + sw = comando (para la telemetria)
        return pct


class SpeedSMC:
    """SMC de TRACCION: sigue posicion y velocidad de la referencia con un solo
    comando de potencia.

    Superficie:  s_v = (v_ref - v) + c*(pos_ref - pos)       (grado relativo 1)
    Modelo nominal del conjunto motor+auto:  dv/dt = (K*u - v)/tau
    Control:     u = (v + tau*(a_ref + c*(v_ref - v)))/K + eta*sat(s_v/phi)

    K [cm/s por %] y tau [s] se identifican con un escalon de potencia (ver el
    .md). eta cubre el error de K y tau (bateria, carga): condicion
    eta*K/tau > cota del termino no modelado.
    """

    def __init__(self, K, tau, c, eta, phi, u_max=100.0):
        self.K = K
        self.tau = tau
        self.c = c          # [1/s]  polo del error de posicion sobre la superficie
        self.eta = eta      # [%]    ganancia conmutada
        self.phi = phi      # [cm/s] capa limite
        self.u_max = u_max
        self.s = 0.0

    def reset(self):
        self.s = 0.0

    def update(self, e_pos, v_ref, v_meas, a_ref=0.0):
        e_v = v_ref - v_meas
        s = e_v + self.c * e_pos
        u_eq = (v_meas + self.tau * (a_ref + self.c * e_v)) / self.K
        u = u_eq + self.eta * sat(s / self.phi)
        if u > self.u_max:
            u = self.u_max
        elif u < -self.u_max:
            u = -self.u_max
        self.s = s
        return u


# ============================================================================
# ---- modulo: main_rs.py
# ============================================================================
# =============================================================================
# SOLUCION DE PROBLEMAS - Conexion Bluetooth (BLE) con el hub
# =============================================================================
# Sintoma: pybricksdev o la extension de VSCode fallan al conectar, con un
#   error tipo "Could not find all requested services" o
#   "BleakCharacteristicNotFoundError: Characteristic ... was not found".
#   Pybricks Code (en el navegador) conecta bien, pero pybricksdev no.
#
# Solucion (la que funciono):
#   1. Cerrar todo lo conectado al hub: Pybricks Code y las pestanas de
#      Chrome. Una sola conexion BLE a la vez.
#   2. Apagar y prender el hub.
#   3. Administrador de dispositivos -> menu Ver -> "Mostrar dispositivos
#      ocultos" -> seccion Bluetooth -> buscar el hub ("Obi Juan's Hub" o
#      similar) -> clic derecho -> Desinstalar el dispositivo.
#   4. Volver a correr pybricksdev. Windows redescubre la tabla GATT de cero.
# =============================================================================
#
# =============================================================================
# PROTOCOLO CON EL DASHBOARD WEB (por stdin / stdout sobre BLE)
# =============================================================================
# El dashboard manda comandos, uno por linea terminada en \n:
#   PATH x,y,h;x,y,h;...            waypoints (x,y en cm, h en grados)
#   PI  kp_pos,ki_pos,kp_st,ki_st   ganancias de los PI
#   VEL v                          velocidad lineal de referencia [cm/s]
#   ALG RS|DUBINS                   elige el planner (default RS)
#   CTRL PI|SMC                     controlador de DIRECCION (default PI)
#   SMC lam,eta,phi,ki,tlead,psi    parametros del SMC (ver smc_control.py)
#   START                          calcula la trayectoria y arranca el ensayo
#   STOP                           frena el auto de inmediato
#   RESET                          pone los sensores (IMU + encoders) a 0
#   BAT                            el hub responde con el estado de bateria
#   PING                           el hub responde RDY (para sincronizar)
#
# El hub responde, una linea por mensaje:
#   RDY                            listo, esperando comandos
#   ACK <texto>                    comando aceptado
#   ERR <texto>                    comando rechazado
#   SEG i n                        arranca el tramo i de n
#   PLAN x,y;x,y;...               trayectoria planificada de un tramo (cm)
#   T t,pr,pv,pe,pc,sr,sv,se,sc,x,y,pP,pI,sP,sI,ey,ss   telemetria (ver mas abajo)
#   END ok|stop|stall|timeout|error   fin del ensayo
#   BAT mv,ma                      bateria: tension [mV] y corriente [mA]
#   LOOP ms ovr                    periodo real medio del lazo [ms/tick] y desbordes (ticks que tardaron mas que LOOP_MS)
#
# Campos de la telemetria T:
#   t  tiempo [s]            pr posicion referencia [cm]   pv posicion real [cm]
#   pe error posicion [cm]   pc comando traccion           sr rumbo ref [grados]
#   sv rumbo real [grados]   se error rumbo [grados]       sc comando direccion
#   x  posicion X global [cm]   y posicion Y global [cm]
#   pP/pI accion traccion P e I    sP/sI accion direccion P e I (con gear)
#   ey error lateral [cm] (odometria, marco de la referencia)   ss variable de deslizamiento s [rad]
#   Con CTRL SMC: sP = control equivalente, sI = termino conmutado (+integral), ambos en % y con signo de sc.
#   pc=clamp(pP+pI), sc=gear*clamp(sP+sI). Si pP+pI o sP+sI exceden el
#   limite, pc/sc saturan pero pP,pI,sP,sI siguen: asi se ve el windup.
# =============================================================================
#
# =============================================================================
# NOTA SOBRE CONVENCION DE SIGNO DEL RUMBO PARA LA VISUALIZACION
# =============================================================================
# El planner (reeds_shepp / dubinsPathPlanning) usa LEFT=-1 y el IMU del hub
# trabaja en la misma convencion ("compass": CW positivo). El lazo de control
# funciona perfecto porque ambos coinciden.
#
# Para INTEGRAR (x, y) con las formulas estandar dx=cos(h)*ds, dy=sin(h)*ds
# necesitamos que el rumbo este en convencion matematica (CCW positivo), si
# no la trayectoria sale espejada contra Y. Por eso al integrar invertimos
# el signo del rumbo en dos lugares:
#   - stream_plan:   thl -= gear * steer_dir * ds / r_turn_min
#   - controlDelVehiculo (odometria viva):   heading_rad = -radians(IMU)
# Estos dos cambios SOLO afectan la grafica del dashboard. El lazo PI sigue
# usando steering_vehicle = hub.imu.heading() crudo, asi que el control no
# se altera.
# =============================================================================



# Modulos estandar de MicroPython para leer stdin. Este es el metodo oficial
# de Pybricks para recibir datos por BLE (ver tutorial "Hub to PC Communication").


ANGLE_TO_CM = 0.045

# Periodo FIJO del lazo de control [ms]. La referencia avanza POR TICK (delta_s =
# h*VELOCITY/1000), asi que la velocidad real es VELOCITY solo si el periodo real del
# lazo es h. Con wait(1) el periodo real salia ~3.5 ms (lectura de sensores + calculo) y
# el auto andaba a ~1/3.5 de VELOCITY. Ahora el lazo espera lo que falta hasta LOOP_MS.
# Condiciones: dividir a 100 (telemetria cada 100 ms) y ser mayor que el tiempo de calculo
# del lazo (la linea LOOP informa el periodo medio y los desbordes; desbordes > 0 => subir).
LOOP_MS = 5
# Periodo real [ms] con el que se sintonizo el PI (medido con el lazo libre: 2.34 ms). Los Ki
# estan definidos POR TICK a ese periodo; se reescalan por LOOP_MS/KI_REF_MS para que el
# efecto por segundo del PI no cambie al fijar el periodo.
KI_REF_MS = 2.34

# Reeds-Shepp y Dubins generan segmentos de transicion de distancia casi nula
# (los giros 't' y 'v' de las curvas CSC suelen dar 0). Si esos segmentos
# llegan al lazo de control, 'position_reference' no avanza (distance ~ 0) y el
# auto queda clavado con accion de traccion 0 hasta el watchdog. Los salteamos.
MIN_SEG_DISTANCE = 1.0   # [cm]


hub = TechnicHub()
steering = Motor(Port.D, Direction.CLOCKWISE)
front = Motor(Port.B, Direction.CLOCKWISE)
rear = Motor(Port.A, Direction.CLOCKWISE)
audi = Car(steering, [front, rear])

run_watch = StopWatch()

# Ganancias de los PI. Arrancan con los valores de controlConstants.py; el
# comando PI las cambia entre ensayos.
GAINS = {'kp_pos': KP_POSITION, 'ki_pos': KI_POSITION,
         'kp_st': KP_STEERING,  'ki_st': KI_STEERING}

# Velocidad lineal de referencia [cm/s]. Arranca con vehicleConstants.py; el
# comando VEL la cambia entre ensayos.
VELOCITY = v

# Algoritmo de planificacion. 'RS' o 'DUBINS'. El comando ALG lo cambia.
ALGORITHM = 'RS'

# Controlador de DIRECCION: 'PI' (rumbo, el de siempre) o 'SMC' (modos deslizantes,
# ver smc_control.py y SMC/SMC_diseno.md). El comando CTRL lo cambia. La traccion
# sigue siendo el PI de posicion en ambos casos.
CONTROLLER = 'PI'
# lam [1/cm], eta [1/cm], phi [rad], ki, tlead [s] (anticipacion de curvatura),
# psi [grados] = angulo de rueda efectivo a +-100 % de direccion (calibrar).
SMC_PARAMS = {'lam': 0.035, 'eta': 0.008, 'phi': 0.05, 'ki': 0.0,
              'tlead': 0.2, 'psi': psi_max}

# Cache de la trayectoria calculada al recibir PATH (para graficarla antes del
# ensayo y no recalcular en START). Se invalida al cambiar PATH o ALG.
LEGS = None
LEGS_TOTAL = 0


# ----------------------------------------------------------------------------
# Lectura de comandos por stdin
# Metodo oficial de Pybricks: usys.stdin + uselect.poll, no bloqueante.
# ----------------------------------------------------------------------------
_poller = poll()
_poller.register(stdin)
_inbuf = ''

def read_command():
    """Arma una linea completa desde stdin sin bloquear. Devuelve la linea
    (str) o None si todavia no llego un fin de linea. Acepta LF, CR o CRLF."""
    global _inbuf
    while _poller.poll(0):
        ch = stdin.buffer.read(1)
        if not ch:
            break
        b = ch[0]
        if b == 10 or b == 13:     # \n o \r  -> fin de linea
            if not _inbuf:         # \r\n o linea vacia: no emitir doble
                continue
            line = _inbuf
            _inbuf = ''
            return line.strip()
        else:
            _inbuf += chr(b)       # ASCII: chr() evita bytes(list) y decode()
    return None


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
    """'kp_pos,ki_pos,kp_st,ki_st' -> actualiza GAINS. Devuelve True si ok."""
    parts = arg.split(',')
    if len(parts) != 4:
        return False
    try:
        GAINS['kp_pos'] = float(parts[0])
        GAINS['ki_pos'] = float(parts[1])
        GAINS['kp_st'] = float(parts[2])
        GAINS['ki_st'] = float(parts[3])
    except:
        return False
    return True


def set_velocity(arg):
    """'<v>' en cm/s -> actualiza VELOCITY. Devuelve True si ok (numero > 0)."""
    global VELOCITY
    try:
        val = float(arg)
    except:
        return False
    if val <= 0:
        return False
    VELOCITY = val
    return True


def set_algorithm(arg):
    """'RS' o 'DUBINS' -> actualiza ALGORITHM. Devuelve True si ok.
    Invalida la trayectoria cacheada (cambia el planner)."""
    global ALGORITHM, LEGS
    arg = arg.strip().upper()
    if arg not in ('RS', 'DUBINS'):
        return False
    ALGORITHM = arg
    LEGS = None
    return True


def set_controller(arg):
    """'PI' o 'SMC' -> actualiza CONTROLLER. Devuelve True si ok."""
    global CONTROLLER
    arg = arg.strip().upper()
    if arg not in ('PI', 'SMC'):
        return False
    CONTROLLER = arg
    return True


def set_smc(arg):
    """'lam,eta,phi,ki,tlead,psi' -> actualiza SMC_PARAMS. Devuelve True si ok."""
    parts = arg.split(',')
    if len(parts) != 6:
        return False
    try:
        lam = float(parts[0]); eta = float(parts[1]); phi = float(parts[2])
        ki = float(parts[3]); tlead = float(parts[4]); psi = float(parts[5])
    except:
        return False
    if lam <= 0 or eta <= 0 or phi <= 0 or ki < 0 or tlead < 0 or psi < 5 or psi > 35:
        return False
    SMC_PARAMS['lam'] = lam
    SMC_PARAMS['eta'] = eta
    SMC_PARAMS['phi'] = phi
    SMC_PARAMS['ki'] = ki
    SMC_PARAMS['tlead'] = tlead
    SMC_PARAMS['psi'] = psi
    return True


def plan_leg(start_pose, end_pose):
    """Planifica un tramo segun el algoritmo seleccionado. Devuelve la lista
    [{distance, steering, gear}, ...] con distancias en cm, o None si no hay
    solucion."""
    if ALGORITHM == 'DUBINS':
        return get_best_dubins(start_pose, end_pose, r_turn_min)
    # Reeds-Shepp (default)
    sp, ep = normalize_start_and_end_point(start_pose, end_pose, r_turn_min)
    best = get_best_path(sp, ep)
    if best is None:
        return None
    return denormalize_distance_in_path(best, r_turn_min)


# ----------------------------------------------------------------------------
# Hardware
# ----------------------------------------------------------------------------
def stop_car():
    """Frena traccion y centra la direccion. Se llama desde STOP, stall,
    timeout y fin de tramo."""
    audi.drive_power(0)
    audi.steer(0)


def resetAllSensors():
    audi.drive_power(0)
    audi.steer(0)
    hub.imu.reset_heading(0)
    front.reset_angle(0)
    rear.reset_angle(0)


def send_battery():
    """Estado de la bateria al dashboard: 'BAT mv,ma'
    mv = tension [mV], ma = corriente de descarga [mA]."""
    try:
        mv = hub.battery.voltage()
        ma = hub.battery.current()
    except:
        mv = 0
        ma = 0
    print("BAT {},{}".format(mv, ma))


def setup():
    hub.light.on(Color.RED)
    hub.imu.settings(1.5, 250)


# ----------------------------------------------------------------------------
# Geometria: integrar la trayectoria planificada de un tramo a coordenadas
# globales (x, y) para dibujarla en el dashboard.
# ----------------------------------------------------------------------------
def make_frame(leg_start):
    """Devuelve to_global(xl, yl): pasa coordenadas locales del tramo
    (origen en leg_start, eje X segun el rumbo inicial) al marco global."""
    th0 = umath.radians(leg_start[2])
    c = umath.cos(th0)
    s = umath.sin(th0)
    x0 = leg_start[0]
    y0 = leg_start[1]
    def to_global(xl, yl):
        return (x0 + xl * c - yl * s, y0 + xl * s + yl * c)
    return to_global


def stream_plan(path, leg_start, step=8.0, batch=14):
    """Integra la trayectoria planificada de un tramo y la manda al
    dashboard en lineas PLAN cortas. No acumula la lista completa de
    puntos: el pico de memoria es de solo "batch" puntos.

    Ojo: el rumbo se acumula con signo invertido (thl -=) para pasar de la
    convencion 'compass' del planner a la matematica estandar usada por
    cos/sin. Esto solo afecta la VISUALIZACION."""
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


# ----------------------------------------------------------------------------
# Control del vehiculo a lo largo de un tramo (leg)
# ----------------------------------------------------------------------------
def saturate(value, limit):
    """Recorta value a [-limit, limit] y lo devuelve entero (los motores
    toman potencia entera)."""
    return int(min(max(value, -limit), limit))


def send_telemetry(t, pos_ref, pos_veh, pos_err, pos_cmd,
                   st_ref_deg, st_veh, st_err, st_cmd,
                   x, y, pP, pI, sP, sI, ey, ss):
    """Imprime una linea T de telemetria al dashboard (formato y campos en
    la cabecera del archivo)."""
    print("T {:.2f},{:.2f},{:.2f},{:.2f},{},{:.2f},{:.2f},{:.2f},{},{:.1f},{:.1f},{:.1f},{:.1f},{:.1f},{:.1f},{:.2f},{:.3f}".format(
        t, pos_ref, pos_veh, pos_err, pos_cmd,
        st_ref_deg, st_veh, st_err, st_cmd,
        x, y, pP, pI, sP, sI, ey, ss))


def controlDelVehiculo(path, leg_start, seg_offset, seg_total, h=LOOP_MS):
    """Maneja el auto por un tramo. Devuelve 'ok', 'stop', 'stall' o
    'timeout'."""
    elapsed_time_max = 30      # [seg] watchdog por tramo
    elapsed_time = 0
    resetAllSensors()

    steering_reference = 0.0
    delta_s = h * VELOCITY / 1000.0

    # Pose de la REFERENCIA en el marco del tramo (misma integracion Euler que la
    # referencia de rumbo/arco): da el error lateral e_y y el atraso e_s para el
    # SMC y para la metrica de error lateral (que se reporta con PI y con SMC).
    x_ref = 0.0
    y_ref = 0.0
    use_smc = (CONTROLLER == 'SMC')
    smc = LateralSMC(L_car, SMC_PARAMS['psi'], SMC_PARAMS['lam'], SMC_PARAMS['eta'],
                     SMC_PARAMS['phi'], SMC_PARAMS['ki'])
    ki_scale = h / KI_REF_MS
    loop_ticks = 0
    overruns = 0
    loop_t0 = run_watch.time()
    next_tick = loop_t0

    # Frame global del tramo, para reportar la posicion (x, y) del auto.
    to_global = make_frame(leg_start)
    x_local = 0.0
    y_local = 0.0

    for i in range(len(path)):
        print("SEG {} {}".format(seg_offset + i + 1, seg_total))

        distance = path[i]['distance']
        steer_dir = path[i]['steering']
        gear = path[i]['gear']

        # Perfil de referencia del tramo (criterio Euler, paso fijo h):
        # cuantizamos el tramo a N_seg ticks enteros del timer y recalculamos
        # el paso de arco ds_seg = distance/N_seg para caer EXACTO en 'distance'
        # en N_seg pasos. No usamos umath.ceil para no depender de el.
        N_seg = int(distance / delta_s)
        if N_seg * delta_s < distance:
            N_seg += 1
        if N_seg < 1:
            N_seg = 1
        ds_seg = distance / N_seg
        ref_ticks = 0
        head_ref_seg0 = steering_reference   # rumbo acumulado al entrar al tramo

        front.reset_angle(0)
        rear.reset_angle(0)
        position_reference = 0.0
        position_vehicle = 0.0
        position_vehicle_prev = 0.0
        steering_I = 0.0
        position_I = 0.0
        smc.reset()
        kappa_ref = -steer_dir / r_turn_min      # curvatura de la referencia (+ = izquierda)
        nxt = path[i + 1] if i + 1 < len(path) else None

        while abs(position_vehicle) <= distance and (elapsed_time / 1000.0) < elapsed_time_max:
            # STOP de emergencia desde el dashboard
            if read_command() == "STOP":
                stop_car()
                return 'stop'

            # Motores bloqueados
            if rear.stalled() or front.stalled():
                stop_car()
                return 'stall'

            # Referencia por Euler con paso fijo y N entero de ticks. En vez de
            # cortar por umbral (que overshoot-ea hasta delta_s y deja un residuo
            # de rumbo que se arrastra al proximo segmento), avanzamos ds_seg
            # exactos N_seg veces y en el ultimo tick clavamos la referencia en
            # su valor final: cierra exacto y no acumula deriva de rumbo.
            if ref_ticks < N_seg:
                ref_ticks += 1
                if ref_ticks < N_seg:
                    position_reference += gear * ds_seg
                    steering_reference += gear * steer_dir * ds_seg / r_turn_min
                else:
                    # ultimo tick del tramo: snap al valor exacto (mata la deriva
                    # por redondeo en float, equivalente al q = qf del resumen).
                    position_reference = gear * distance
                    steering_reference = head_ref_seg0 + gear * steer_dir * distance / r_turn_min
                # pose de la referencia (convencion matematica, igual que la odometria)
                th_r = -steering_reference
                x_ref += gear * ds_seg * umath.cos(th_r)
                y_ref += gear * ds_seg * umath.sin(th_r)

            # Estado del vehiculo
            position_vehicle = 0.5 * (front.angle() + rear.angle()) * ANGLE_TO_CM
            steering_vehicle = hub.imu.heading()

            # Errores
            position_error = position_reference - position_vehicle
            steering_error = umath.degrees(steering_reference) - steering_vehicle

            # Odometria (x, y) del auto. El rumbo del IMU se invierte de signo SOLO
            # para esta integracion (convencion matematica estandar).
            ds = position_vehicle - position_vehicle_prev
            position_vehicle_prev = position_vehicle
            heading_rad = -umath.radians(steering_vehicle)
            x_local += ds * umath.cos(heading_rad)
            y_local += ds * umath.sin(heading_rad)

            # Errores de traza en el marco de la referencia: e_s (atraso a lo largo),
            # e_y (lateral, + = izquierda).
            th_r = -steering_reference
            c_r = umath.cos(th_r)
            s_r = umath.sin(th_r)
            dx_r = x_local - x_ref
            dy_r = y_local - y_ref
            e_s = c_r * dx_r + s_r * dy_r
            e_y = -s_r * dx_r + c_r * dy_r

            # --- Accion de control de POSICION (traccion), PI ---
            position_P = GAINS['kp_pos'] * position_error
            position_I += GAINS['ki_pos'] * ki_scale * position_error
            position_command = saturate(position_P + position_I, max_drive_power)

            # --- Accion de control de DIRECCION (steering) ---
            if use_smc:
                # SMC: superficie s = e_g + lam*gear*e_y (ver smc_control.py). Sale en
                # convencion matematica (+ = izquierda); el hub usa compass (CW+): signo -.
                k_lead = kappa_ref
                if (nxt is not None and nxt['gear'] == gear and
                        distance - abs(position_vehicle) < VELOCITY * SMC_PARAMS['tlead']):
                    k_lead = -nxt['steering'] / r_turn_min    # anticipa la curvatura que viene
                pct = smc.update(gear, umath.radians(steering_error), e_y, e_s,
                                 kappa_ref, abs(ds), k_lead)
                steering_command = saturate(-pct, max_angle_power)
                # Para las barras del dashboard: P = control equivalente, I = conmutado
                # (ambos con el signo de steering_command; la telemetria los multiplica
                # por gear, por eso se divide aca).
                steering_P = -gear * smc.pct_eq
                steering_I = -gear * smc.pct_sw
            else:
                steering_P = GAINS['kp_st'] * steering_error
                steering_I += GAINS['ki_st'] * ki_scale * steering_error
                steering_command = gear * saturate(steering_P + steering_I, max_angle_power)

            audi.steer(steering_command)
            audi.drive_power(position_command)

            # Lazo a periodo fijo: espera lo que falta hasta el proximo tick. Si el calculo
            # tardo mas que h, resincroniza y cuenta el desborde.
            next_tick += h
            rem = next_tick - run_watch.time()
            if rem > 0:
                wait(rem)
            elif rem < 0:
                # Tick pasado de tiempo: se cuenta y NO se resincroniza; los ticks siguientes
                # lo recuperan sin esperar, asi el periodo MEDIO queda en h (y la velocidad
                # real = VELOCITY). Solo si el atraso supera 4*h se resincroniza de golpe.
                overruns += 1
                if rem < -4 * h:
                    next_tick = run_watch.time()
            elapsed_time += h
            loop_ticks += 1

            # Telemetria al dashboard, cada 100 ms
            if elapsed_time % 100 == 0:
                x_global, y_global = to_global(x_local, y_local)
                send_telemetry(
                    run_watch.time() / 1000.0,
                    position_reference, position_vehicle, position_error, position_command,
                    umath.degrees(steering_reference), steering_vehicle, steering_error, steering_command,
                    x_global, y_global,
                    position_P, position_I, gear * steering_P, gear * steering_I,
                    e_y, smc.s if use_smc else 0.0)

        # Watchdog de tiempo
        if (elapsed_time / 1000.0) >= elapsed_time_max:
            stop_car()
            return 'timeout'

    stop_car()
    if loop_ticks > 0:
        print("LOOP {:.2f} {}".format((run_watch.time() - loop_t0) / loop_ticks, overruns))
    return 'ok'


# ----------------------------------------------------------------------------
# Calculo de la trayectoria y envio al dashboard
# ----------------------------------------------------------------------------
def build_legs(PATH):
    """Calcula los legs (con filtro de segmentos nulos) de todo el PATH segun
    el algoritmo elegido. Devuelve (legs, seg_total) o (None, 0) si algun
    tramo no tiene solucion."""
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


def send_plan(legs, PATH):
    """Manda la trayectoria planificada de cada tramo al dashboard (PLAN)."""
    for i in range(len(legs)):
        gc.collect()
        stream_plan(legs[i], PATH[i])


# ----------------------------------------------------------------------------
# Ejecucion de la trayectoria completa
# ----------------------------------------------------------------------------
def run_trajectory(PATH):
    global LEGS, LEGS_TOTAL
    run_watch.reset()
    hub.light.on(Color.ORANGE)

    # Reusar la trayectoria calculada al recibir PATH; si no hay, calcularla.
    if LEGS is None:
        LEGS, LEGS_TOTAL = build_legs(PATH)
        if LEGS is None:
            print("ERR sin solucion {}".format(ALGORITHM))
            print("END error")
            hub.light.on(Color.RED)
            return
        # Recalculado aca (no hubo preview al recibir PATH): mandarlo ahora.
        send_plan(LEGS, PATH)
    legs = LEGS
    seg_total = LEGS_TOTAL

    # Manejar tramo por tramo
    gc.collect()
    seg_offset = 0
    for i in range(len(legs)):
        status = controlDelVehiculo(legs[i], PATH[i], seg_offset, seg_total)
        seg_offset += len(legs[i])
        if status != 'ok':
            print("END " + status)
            hub.light.on(Color.RED)
            return

    print("END ok")
    hub.light.on(Color.GREEN)


# ----------------------------------------------------------------------------
# Bucle principal: escucha comandos del dashboard
# ----------------------------------------------------------------------------
def main():
    global LEGS, LEGS_TOTAL
    setup()
    PATH = None
    print("RDY")
    print("VER 15 rs-dubins-pi-smc-loopfijo")
    send_battery()

    while True:
        line = read_command()
        if line is None:
            wait(20)
            continue
        if line == "":
            continue

        if line.startswith("PATH "):
            poses = parse_path(line[5:])
            if poses and len(poses) >= 2:
                PATH = poses
                LEGS, LEGS_TOTAL = build_legs(PATH)
                if LEGS is None:
                    print("ERR sin solucion {}".format(ALGORITHM))
                else:
                    print("ACK PATH {}".format(len(PATH)))
                    send_plan(LEGS, PATH)
            else:
                PATH = None
                LEGS = None
                print("ERR PATH invalido (minimo 2 poses x,y,h)")

        elif line.startswith("PI "):
            if set_gains(line[3:]):
                print("ACK PI {},{},{},{}".format(
                    GAINS['kp_pos'], GAINS['ki_pos'], GAINS['kp_st'], GAINS['ki_st']))
            else:
                print("ERR PI invalido (4 numeros)")

        elif line.startswith("VEL "):
            if set_velocity(line[4:]):
                print("ACK VEL {}".format(VELOCITY))
            else:
                print("ERR VEL invalido (1 numero > 0)")

        elif line.startswith("ALG "):
            if set_algorithm(line[4:]):
                print("ACK ALG {}".format(ALGORITHM))
            else:
                print("ERR ALG invalido (RS o DUBINS)")

        elif line.startswith("CTRL "):
            if set_controller(line[5:]):
                print("ACK CTRL {}".format(CONTROLLER))
            else:
                print("ERR CTRL invalido (PI o SMC)")

        elif line.startswith("SMC "):
            if set_smc(line[4:]):
                print("ACK SMC {},{},{},{},{},{}".format(
                    SMC_PARAMS['lam'], SMC_PARAMS['eta'], SMC_PARAMS['phi'],
                    SMC_PARAMS['ki'], SMC_PARAMS['tlead'], SMC_PARAMS['psi']))
            else:
                print("ERR SMC invalido (lam,eta,phi,ki,tlead,psi; todos > 0 salvo ki y tlead >= 0; psi 5..35)")

        elif line == "START":
            if not PATH:
                print("ERR sin PATH cargado")
            else:
                print("ACK START {}".format(ALGORITHM))
                run_trajectory(PATH)
                send_battery()
                print("RDY")

        elif line == "STOP":
            stop_car()
            print("ACK STOP")

        elif line == "RESET":
            resetAllSensors()
            send_battery()
            print("ACK RESET")

        elif line == "BAT":
            send_battery()

        elif line == "PING":
            print("RDY")

        else:
            print("ERR comando desconocido: " + line)


if __name__ == "__main__":
    main()
