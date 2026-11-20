# ============================================================================
# main_bundle.py - GENERADO por SMC/make_bundle.py (2026-11-20 01:20). NO EDITAR A MANO:
# editar los .py originales y volver a correr el script.
# ============================================================================
import umath
import gc
from pybricks.hubs import TechnicHub
from pybricks.parameters import Color, Direction, Port
from pybricks.tools import StopWatch
from pybricks.robotics import Car
from pybricks.pupdevices import Motor
from usys import stdin
from uselect import poll
from pybricks.parameters import Color
from pybricks.tools import wait


# ============================================================================
# ---- modulo: vehicleConstants.py
# ============================================================================
L_car = 24.0
psi_max = 20.0
v = 40.0
r_turn_min = 80.0
max_drive_power = 100
max_angle_power = 100


# ============================================================================
# ---- modulo: controlConstants.py
# ============================================================================
KP_POSITION = 50.0
KI_POSITION = 0.01
KP_STEERING = 50.0
KI_STEERING = 0.01


# ============================================================================
# ---- modulo: reeds_shepp.py
# ============================================================================
FORWARD = 1
BACKWARD = -1
LEFT = -1
RIGHT = 1
STRAIGHT = 0
def denormalize_distance_in_path(path, r_turn_min):
    for p in path:
        p['distance'] = p['distance'] * r_turn_min
    return path
def normalize_start_and_end_point(start_point_original, end_point_original, r_turn_min):
    start_point = [0, 0, 0]
    end_point = [0, 0, 0]
    start_point[0] = start_point_original[0] / r_turn_min
    start_point[1] = start_point_original[1] / r_turn_min
    start_point[2] = umath.radians(start_point_original[2]) 
    end_point[0] = end_point_original[0] / r_turn_min
    end_point[1] = end_point_original[1] / r_turn_min
    end_point[2] =  umath.radians(end_point_original[2]) 
    return start_point, end_point
def R(x, y):
    u = umath.sqrt(x**2 + y**2)
    t = umath.atan2(y, x)
    return u, t
def M(theta):
    theta = umath.fmod(theta, 2 * umath.pi)
    if theta < -umath.pi:
        theta += 2 * umath.pi
    elif theta >= umath.pi:
        theta -= 2 * umath.pi
    return theta
def create_path_element(distance, steering, gear):
    if distance < 0:
        gear = -gear
    return {"distance": abs(distance), "steering": steering, "gear": gear}
def change_of_basis(start_point, end_point):
    x, y, theta = start_point
    x_end, y_end, theta_end = end_point
    dx = x_end - x
    dy = y_end - y
    dtheta = M(theta_end - theta)
    x = dx * umath.cos(theta) + dy * umath.sin(theta)
    y = -dx * umath.sin(theta) + dy * umath.cos(theta)
    return x, y, dtheta
def path1(x, y, phi):
    path = []
    u, t = R(x - umath.sin(phi), y - 1 + umath.cos(phi))
    v = M(phi - t)
    path.append(create_path_element(t, LEFT, FORWARD))
    path.append(create_path_element(u, STRAIGHT, FORWARD))
    path.append(create_path_element(v, LEFT, FORWARD))
    return path
def path2(x, y, phi):
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
    element["gear"] *= -1
    return element
def reverse_steering(element):
    element["steering"] *= -1
    return element
def reflect(path):
    new_path = []
    for element in path:
        new_path.append(reverse_steering(element.copy()))
    return new_path
def timeflip(path):
    new_path = []
    for element in path:
        new_path.append(reverse_gear(element.copy()))
    return new_path
def get_all_paths(start_point, end_point):
    path_fns = [path1, path2, path3, path4, path5, path6,
                path7, path8, path9, path10, path11, path12]
    paths = []
    x, y, theta = change_of_basis(start_point, end_point)
    for get_path in path_fns:
        var1 = get_path(x, y, theta)
        var2 = timeflip(get_path(-x, y, -theta))
        var3 = reflect(get_path(x, -y, -theta))
        var4 = reflect(timeflip(get_path(-x, -y, theta)))
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
        path_distance = 0
        for p in path:
            path_distance += p['distance']
        if path_distance < min_distance:
            min_distance = path_distance
            min_distance_index = i
    return min_distance_index
def get_best_path(start_point, end_point):
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
def _arc_signed(theta_arc, turn_dir):
    if turn_dir == 'L' and theta_arc < 0.0:
        theta_arc += 2.0 * umath.pi
    elif turn_dir == 'R' and theta_arc > 0.0:
        theta_arc -= 2.0 * umath.pi
    return theta_arc
def _tangents_CSC(c1, c2, r, path_type):
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
def get_best_dubins(start_pose, end_pose, r_turn_min):
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


# ============================================================================
# ---- modulo: smc_control.py
# ============================================================================
try:
    import umath as _m
except ImportError:
    import math as _m
def sat(x):
    if x > 1.0:
        return 1.0
    if x < -1.0:
        return -1.0
    return x
def wrap_pi(a):
    while a >= _m.pi:
        a -= 2.0 * _m.pi
    while a < -_m.pi:
        a += 2.0 * _m.pi
    return a
class LateralSMC:
    def __init__(self, L, delta_max_deg, lam, eta, phi, ki=0.0):
        self.L = L
        self.delta_max = _m.radians(delta_max_deg)
        self.lam = lam
        self.eta = eta
        self.phi = phi
        self.ki = ki
        self.z = 0.0
        self.s = 0.0
        self.pct_eq = 0.0
        self.pct_sw = 0.0
    def reset(self):
        self.z = 0.0
        self.s = 0.0
        self.pct_eq = 0.0
        self.pct_sw = 0.0
    def update(self, gear, e_theta, e_y, e_s, kappa_ref, ds_abs, kappa_ff=None):
        g = 1.0 if gear >= 0 else -1.0
        e_g = wrap_pi(e_theta - kappa_ref * e_s)
        s = e_g + self.lam * g * e_y
        if kappa_ff is None:
            kappa_ff = kappa_ref
        k_eq = kappa_ff * _m.cos(e_g) / (1.0 - kappa_ref * e_y) - g * self.lam * _m.sin(e_g)
        sw = self.eta * sat(s / self.phi)
        if self.ki > 0.0 and abs(s) < self.phi:
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
        self.pct_sw = pct - self.pct_eq
        return pct
class SpeedSMC:
    def __init__(self, K, tau, c, eta, phi, u_max=100.0):
        self.K = K
        self.tau = tau
        self.c = c
        self.eta = eta
        self.phi = phi
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
# ---- modulo: settings.py
# ============================================================================
LOOP_MS = 5
KI_REF_MS = 2.34
class Settings:
    def __init__(self):
        self.velocity = v
        self.profile = 'TRAP'
        self.accel = 80.0
        self.algorithm = 'RS'
        self.controller = 'PI'
        self.gains = {'kp_pos': KP_POSITION, 'ki_pos': KI_POSITION,
                      'kp_st': KP_STEERING,  'ki_st': KI_STEERING}
        self.smc = {'lam': 0.035, 'eta': 0.008, 'phi': 0.05, 'ki': 0.0,
                    'tlead': 0.2, 'psi': psi_max}
        self.legs = None
        self.legs_total = 0
cfg = Settings()
def parse_path(arg):
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
    try:
        val = float(arg)
    except:
        return False
    if val <= 0:
        return False
    cfg.velocity = val
    return True
def set_profile(arg):
    name = arg.strip().upper()
    if name != 'CONST' and name != 'TRAP':
        return False
    cfg.profile = name
    return True
def set_accel(arg):
    try:
        val = float(arg)
    except:
        return False
    if val < 0:
        return False
    cfg.accel = val
    return True
def set_algorithm(arg):
    arg = arg.strip().upper()
    if arg not in ('RS', 'DUBINS'):
        return False
    cfg.algorithm = arg
    cfg.legs = None
    return True
def set_controller(arg):
    arg = arg.strip().upper()
    if arg not in ('PI', 'SMC'):
        return False
    cfg.controller = arg
    return True
def set_smc(arg):
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
    cfg.smc['lam'] = lam
    cfg.smc['eta'] = eta
    cfg.smc['phi'] = phi
    cfg.smc['ki'] = ki
    cfg.smc['tlead'] = tlead
    cfg.smc['psi'] = psi
    return True


# ============================================================================
# ---- modulo: velocity_profile.py
# ============================================================================
V_MIN_REF = 2.0
class ConstantProfile:
    def __init__(self, distance, v_max, h_ms):
        step = h_ms * v_max / 1000.0
        n = int(distance / step)
        if n * step < distance:
            n += 1
        if n < 1:
            n = 1
        self.n = n
        self.ds = distance / n
        self.ticks = 0
    def next(self):
        self.ticks += 1
        return self.ds, self.ticks >= self.n
class TrapezoidProfile:
    def __init__(self, distance, v0, v1, v_max, accel, h_ms):
        self.distance = distance
        self.v1 = v1
        self.v_max = v_max
        self.accel = accel
        self.h_ms = h_ms
        self.v = v0
        self.s = 0.0
    def next(self):
        remaining = self.distance - self.s
        v_up = self.v + self.accel * self.h_ms / 1000.0
        v_brake = umath.sqrt(self.v1 * self.v1 + 2.0 * self.accel * remaining)
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
    if kind == 'TRAP' and accel > 0:
        v0 = v_max if prev_gear == gear else 0.0
        v1 = v_max if next_gear == gear else 0.0
        return TrapezoidProfile(distance, v0, v1, v_max, accel, h_ms)
    return ConstantProfile(distance, v_max, h_ms)


# ============================================================================
# ---- modulo: reference.py
# ============================================================================
class Reference:
    def __init__(self):
        self.heading = 0.0
        self.x = 0.0
        self.y = 0.0
        self.position = 0.0
        self.done = True
        self._heading0 = 0.0
        self._distance = 0.0
        self._steer_dir = 0
        self._gear = 1
        self._profile = None
    def start_segment(self, distance, steer_dir, gear, profile):
        self._heading0 = self.heading
        self._distance = distance
        self._steer_dir = steer_dir
        self._gear = gear
        self._profile = profile
        self.position = 0.0
        self.done = False
    def advance(self):
        if self.done:
            return
        gear = self._gear
        ds, last = self._profile.next()
        if not last:
            self.position += gear * ds
            self.heading += gear * self._steer_dir * ds / r_turn_min
        else:
            self.position = gear * self._distance
            self.heading = self._heading0 + gear * self._steer_dir * self._distance / r_turn_min
            self.done = True
        th = -self.heading
        self.x += gear * ds * umath.cos(th)
        self.y += gear * ds * umath.sin(th)


# ============================================================================
# ---- modulo: odometry.py
# ============================================================================
class Odometry:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self._pos_prev = 0.0
    def reset_segment(self):
        self._pos_prev = 0.0
    def update(self, position_vehicle, heading_deg):
        ds = position_vehicle - self._pos_prev
        self._pos_prev = position_vehicle
        heading_rad = -umath.radians(heading_deg)
        self.x += ds * umath.cos(heading_rad)
        self.y += ds * umath.sin(heading_rad)
        return ds
    def tracking_errors(self, ref):
        th_r = -ref.heading
        c_r = umath.cos(th_r)
        s_r = umath.sin(th_r)
        dx = self.x - ref.x
        dy = self.y - ref.y
        e_s = c_r * dx + s_r * dy
        e_y = -s_r * dx + c_r * dy
        return e_s, e_y
def make_frame(leg_start):
    th0 = umath.radians(leg_start[2])
    c = umath.cos(th0)
    s = umath.sin(th0)
    x0 = leg_start[0]
    y0 = leg_start[1]
    def to_global(xl, yl):
        return (x0 + xl * c - yl * s, y0 + xl * s + yl * c)
    return to_global


# ============================================================================
# ---- modulo: hardware.py
# ============================================================================
ANGLE_TO_CM = 0.045
hub = TechnicHub()
steering = Motor(Port.D, Direction.CLOCKWISE)
front = Motor(Port.B, Direction.CLOCKWISE)
rear = Motor(Port.A, Direction.CLOCKWISE)
audi = Car(steering, [front, rear])
run_watch = StopWatch()
def setup():
    hub.light.on(Color.RED)
    hub.imu.settings(1.5, 250)
def stop_car():
    audi.drive_power(0)
    audi.steer(0)
def reset_sensors():
    audi.drive_power(0)
    audi.steer(0)
    hub.imu.reset_heading(0)
    front.reset_angle(0)
    rear.reset_angle(0)
def reset_encoders():
    front.reset_angle(0)
    rear.reset_angle(0)
def read_position():
    return 0.5 * (front.angle() + rear.angle()) * ANGLE_TO_CM
def read_heading():
    return hub.imu.heading()
def motors_stalled():
    return rear.stalled() or front.stalled()
def battery():
    try:
        return hub.battery.voltage(), hub.battery.current()
    except:
        return 0, 0


# ============================================================================
# ---- modulo: comms.py
# ============================================================================
_poller = poll()
_poller.register(stdin)
_inbuf = ''
def read_command():
    global _inbuf
    while _poller.poll(0):
        ch = stdin.buffer.read(1)
        if not ch:
            break
        b = ch[0]
        if b == 10 or b == 13:
            if not _inbuf:
                continue
            line = _inbuf
            _inbuf = ''
            return line.strip()
        else:
            _inbuf += chr(b)
    return None
def send_battery():
    mv, ma = battery()
    print("BAT {},{}".format(mv, ma))
def send_telemetry(t, pos_ref, pos_veh, pos_err, pos_cmd,
                   st_ref_deg, st_veh, st_err, st_cmd,
                   x, y, pP, pI, sP, sI, ey, ss):
    print("T {:.2f},{:.2f},{:.2f},{:.2f},{},{:.2f},{:.2f},{:.2f},{},{:.1f},{:.1f},{:.1f},{:.1f},{:.1f},{:.1f},{:.2f},{:.3f}".format(
        t, pos_ref, pos_veh, pos_err, pos_cmd,
        st_ref_deg, st_veh, st_err, st_cmd,
        x, y, pP, pI, sP, sI, ey, ss))


# ============================================================================
# ---- modulo: trajectory.py
# ============================================================================
MIN_SEG_DISTANCE = 1.0
def plan_leg(start_pose, end_pose):
    if cfg.algorithm == 'DUBINS':
        return get_best_dubins(start_pose, end_pose, r_turn_min)
    sp, ep = normalize_start_and_end_point(start_pose, end_pose, r_turn_min)
    best = get_best_path(sp, ep)
    if best is None:
        return None
    return denormalize_distance_in_path(best, r_turn_min)
def build_legs(PATH):
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
    to_global = make_frame(leg_start)
    xl = 0.0
    yl = 0.0
    thl = 0.0
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
    for i in range(len(legs)):
        gc.collect()
        stream_plan(legs[i], PATH[i])


# ============================================================================
# ---- modulo: controllers.py
# ============================================================================
def saturate(value, limit):
    return int(min(max(value, -limit), limit))
def integrate(integral, p, delta, limit):
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
    return PI('kp_pos', 'ki_pos', max_drive_power, h)
class SteeringPI:
    def __init__(self, h=LOOP_MS):
        self.pi = PI('kp_st', 'ki_st', max_angle_power, h)
        self.s = 0.0
    def reset(self):
        self.pi.reset()
    def update(self, gear, steering_error, *unused):
        command, p, i = self.pi.update(steering_error)
        return gear * command, p, i
class SteeringSMC:
    def __init__(self, *unused):
        sm = cfg.smc
        self.smc = LateralSMC(L_car, sm['psi'], sm['lam'], sm['eta'], sm['phi'], sm['ki'])
        self.s = 0.0
    def reset(self):
        self.smc.reset()
    def update(self, gear, steering_error, e_y, e_s, ds, kappa_ref, k_lead):
        pct = self.smc.update(gear, umath.radians(steering_error), e_y, e_s,
                              kappa_ref, abs(ds), k_lead)
        self.s = self.smc.s
        command = saturate(-pct, max_angle_power)
        return command, -gear * self.smc.pct_eq, -gear * self.smc.pct_sw
def curvature_preview(nxt, gear, distance, travelled, kappa_ref):
    if (nxt is not None and nxt['gear'] == gear and
            distance - travelled < cfg.velocity * cfg.smc['tlead']):
        return -nxt['steering'] / r_turn_min
    return kappa_ref
def make_steering_controller(kind):
    if kind == 'SMC':
        return SteeringSMC()
    return SteeringPI()


# ============================================================================
# ---- modulo: main_rs.py
# ============================================================================
def controlDelVehiculo(path, leg_start, seg_offset, seg_total, h=LOOP_MS):
    elapsed_time_max = 30
    elapsed_time = 0
    reset_sensors()
    ref = Reference()
    odo = Odometry()
    pos_ctrl = make_position_controller(h)
    steer_ctrl = make_steering_controller(cfg.controller)
    to_global = make_frame(leg_start)
    loop_ticks = 0
    overruns = 0
    loop_t0 = run_watch.time()
    next_tick = loop_t0
    for i in range(len(path)):
        print("SEG {} {}".format(seg_offset + i + 1, seg_total))
        distance = path[i]['distance']
        steer_dir = path[i]['steering']
        gear = path[i]['gear']
        prev_gear = path[i - 1]['gear'] if i > 0 else None
        nxt = path[i + 1] if i + 1 < len(path) else None
        next_gear = nxt['gear'] if nxt is not None else None
        profile = make_profile(cfg.profile, distance, gear, prev_gear, next_gear,
                               cfg.velocity, cfg.accel, h)
        ref.start_segment(distance, steer_dir, gear, profile)
        reset_encoders()
        odo.reset_segment()
        position_vehicle = 0.0
        pos_ctrl.reset()
        steer_ctrl.reset()
        kappa_ref = -steer_dir / r_turn_min
        while abs(position_vehicle) <= distance and (elapsed_time / 1000.0) < elapsed_time_max:
            if read_command() == "STOP":
                stop_car()
                return 'stop'
            if motors_stalled():
                stop_car()
                return 'stall'
            ref.advance()
            position_vehicle = read_position()
            steering_vehicle = read_heading()
            position_error = ref.position - position_vehicle
            steering_error = umath.degrees(ref.heading) - steering_vehicle
            ds = odo.update(position_vehicle, steering_vehicle)
            e_s, e_y = odo.tracking_errors(ref)
            position_command, position_P, position_I = pos_ctrl.update(position_error)
            k_lead = curvature_preview(nxt, gear, distance, abs(position_vehicle), kappa_ref)
            steering_command, steering_P, steering_I = steer_ctrl.update(
                gear, steering_error, e_y, e_s, ds, kappa_ref, k_lead)
            audi.steer(steering_command)
            audi.drive_power(position_command)
            next_tick += h
            rem = next_tick - run_watch.time()
            if rem > 0:
                wait(rem)
            elif rem < 0:
                overruns += 1
                if rem < -4 * h:
                    next_tick = run_watch.time()
            elapsed_time += h
            loop_ticks += 1
            if elapsed_time % 100 == 0:
                x_global, y_global = to_global(odo.x, odo.y)
                send_telemetry(
                    run_watch.time() / 1000.0,
                    ref.position, position_vehicle, position_error, position_command,
                    umath.degrees(ref.heading), steering_vehicle, steering_error, steering_command,
                    x_global, y_global,
                    position_P, position_I, gear * steering_P, gear * steering_I,
                    e_y, steer_ctrl.s)
        if (elapsed_time / 1000.0) >= elapsed_time_max:
            stop_car()
            return 'timeout'
    stop_car()
    if loop_ticks > 0:
        print("LOOP {:.2f} {}".format((run_watch.time() - loop_t0) / loop_ticks, overruns))
    return 'ok'
def run_trajectory(PATH):
    run_watch.reset()
    hub.light.on(Color.ORANGE)
    if cfg.legs is None:
        cfg.legs, cfg.legs_total = build_legs(PATH)
        if cfg.legs is None:
            print("ERR sin solucion {}".format(cfg.algorithm))
            print("END error")
            hub.light.on(Color.RED)
            return
        send_plan(cfg.legs, PATH)
    legs = cfg.legs
    seg_total = cfg.legs_total
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
def main():
    setup()
    PATH = None
    print("RDY")
    print("VER 16 modular")
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
                cfg.legs, cfg.legs_total = build_legs(PATH)
                if cfg.legs is None:
                    print("ERR sin solucion {}".format(cfg.algorithm))
                else:
                    print("ACK PATH {}".format(len(PATH)))
                    send_plan(cfg.legs, PATH)
            else:
                PATH = None
                cfg.legs = None
                print("ERR PATH invalido (minimo 2 poses x,y,h)")
        elif line.startswith("PI "):
            if set_gains(line[3:]):
                print("ACK PI {},{},{},{}".format(
                    cfg.gains['kp_pos'], cfg.gains['ki_pos'], cfg.gains['kp_st'], cfg.gains['ki_st']))
            else:
                print("ERR PI invalido (4 numeros)")
        elif line.startswith("PROF "):
            if set_profile(line[5:]):
                print("ACK PROF {}".format(cfg.profile))
            else:
                print("ERR PROF invalido (CONST o TRAP)")
        elif line.startswith("ACC "):
            if set_accel(line[4:]):
                print("ACK ACC {}".format(cfg.accel))
            else:
                print("ERR ACC invalido (1 numero >= 0)")
        elif line.startswith("VEL "):
            if set_velocity(line[4:]):
                print("ACK VEL {}".format(cfg.velocity))
            else:
                print("ERR VEL invalido (1 numero > 0)")
        elif line.startswith("ALG "):
            if set_algorithm(line[4:]):
                print("ACK ALG {}".format(cfg.algorithm))
            else:
                print("ERR ALG invalido (RS o DUBINS)")
        elif line.startswith("CTRL "):
            if set_controller(line[5:]):
                print("ACK CTRL {}".format(cfg.controller))
            else:
                print("ERR CTRL invalido (PI o SMC)")
        elif line.startswith("SMC "):
            if set_smc(line[4:]):
                print("ACK SMC {},{},{},{},{},{}".format(
                    cfg.smc['lam'], cfg.smc['eta'], cfg.smc['phi'],
                    cfg.smc['ki'], cfg.smc['tlead'], cfg.smc['psi']))
            else:
                print("ERR SMC invalido (lam,eta,phi,ki,tlead,psi; todos > 0 salvo ki y tlead >= 0; psi 5..35)")
        elif line == "START":
            if not PATH:
                print("ERR sin PATH cargado")
            else:
                print("ACK START {}".format(cfg.algorithm))
                run_trajectory(PATH)
                send_battery()
                print("RDY")
        elif line == "STOP":
            stop_car()
            print("ACK STOP")
        elif line == "RESET":
            reset_sensors()
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
