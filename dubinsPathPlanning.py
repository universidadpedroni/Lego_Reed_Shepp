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

import umath
import gc

# Convencion compartida con reeds_shepp para que el controlador del auto
# trate ambos planners por igual.
from reeds_shepp import LEFT, RIGHT, STRAIGHT, FORWARD


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
def run_test():
    from vehicleConstants import r_turn_min
    start = (0.0, 0.0, 0.0)
    end = (120.0, 0.0, 0.0)
    path = get_best_dubins(start, end, r_turn_min)
    print("r_turn_min =", r_turn_min)
    print("path:", path)
    if path:
        print("longitud total:", sum(p['distance'] for p in path))


if __name__ == "__main__":
    run_test()
