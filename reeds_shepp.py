import umath
import gc
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




def run_test():
    x = 120.0
    y = 60.0
    phi = umath.radians(-90)  # las funciones path* esperan el angulo en radianes
    
    # Llamada a todas las funciones de path
    for i in range(1, 13):
        path = globals()[f'path{i}'](x, y, phi)
        print(f"path{i}: {path}")

if __name__ == "__main__":
    run_test()