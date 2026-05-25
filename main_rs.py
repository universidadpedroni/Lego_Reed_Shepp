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
#   PID kp_pos,ki_pos,kp_st,ki_st   ganancias de los PID
#   START                          calcula la trayectoria y arranca el ensayo
#   STOP                           frena el auto de inmediato
#   PING                           el hub responde RDY (para sincronizar)
#
# El hub responde, una linea por mensaje:
#   RDY                            listo, esperando comandos
#   ACK <texto>                    comando aceptado
#   ERR <texto>                    comando rechazado
#   SEG i n                        arranca el tramo i de n
#   PLAN x,y;x,y;...               trayectoria planificada de un tramo (cm)
#   T t,pr,pv,pe,pc,sr,sv,se,sc,x,y   muestra de telemetria (ver mas abajo)
#   END ok|stop|stall|timeout|error   fin del ensayo
#
# Campos de la telemetria T:
#   t  tiempo [s]            pr posicion referencia [cm]   pv posicion real [cm]
#   pe error posicion [cm]   pc comando traccion           sr rumbo ref [grados]
#   sv rumbo real [grados]   se error rumbo [grados]       sc comando direccion
#   x  posicion X global [cm]   y posicion Y global [cm]
# =============================================================================

from pybricks.hubs import TechnicHub
from pybricks.parameters import Color, Direction, Port
from pybricks.tools import wait, StopWatch
from pybricks.robotics import Car
from pybricks.pupdevices import Motor

from reeds_shepp import get_best_path, normalize_start_and_end_point, denormalize_distance_in_path
from vehicleConstants import v, max_angle_power, max_drive_power, r_turn_min
from controlConstants import KP_POSITION, KI_POSITION, KP_STEERING, KI_STEERING
import umath
import gc

# Modulos estandar de MicroPython para leer stdin. Este es el metodo oficial
# de Pybricks para recibir datos por BLE (ver tutorial "Hub to PC Communication").
from usys import stdin
from uselect import poll


CM_TO_ANGLE = 22       # 3600 grados / 162 cm
ANGLE_TO_CM = 0.045


hub = TechnicHub()
steering = Motor(Port.D, Direction.CLOCKWISE)
front = Motor(Port.B, Direction.CLOCKWISE)
rear = Motor(Port.A, Direction.CLOCKWISE)
audi = Car(steering, [front, rear])

run_watch = StopWatch()

# Ganancias de los PID. Arrancan con los valores de controlConstants.py; el
# comando PID las cambia entre ensayos.
GAINS = {'kp_pos': KP_POSITION, 'ki_pos': KI_POSITION,
         'kp_st': KP_STEERING,  'ki_st': KI_STEERING}


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


# ----------------------------------------------------------------------------
# Hardware
# ----------------------------------------------------------------------------
def resetAllSensors():
    audi.steer(0)
    hub.imu.reset_heading(0)
    front.reset_angle(0)
    rear.reset_angle(0)


def setup():
    hub.light.on(Color.RED)
    hub.imu.settings(1.5, 250)


# ----------------------------------------------------------------------------
# Geometria: integrar la trayectoria planificada de un tramo a coordenadas
# globales (x, y) para dibujarla en el dashboard.
# ----------------------------------------------------------------------------
def stream_plan(path, leg_start, step=8.0, batch=14):
    """Integra la trayectoria planificada de un tramo y la manda al
    dashboard en lineas PLAN cortas. No acumula la lista completa de
    puntos: el pico de memoria es de solo "batch" puntos."""
    X0 = leg_start[0]
    Y0 = leg_start[1]
    TH0 = umath.radians(leg_start[2])
    cos0 = umath.cos(TH0)
    sin0 = umath.sin(TH0)

    xl = 0.0       # posicion local dentro del tramo
    yl = 0.0
    thl = 0.0      # rumbo local (arranca en 0)
    parts = ["{:.1f},{:.1f}".format(X0, Y0)]

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
            thl += gear * steer_dir * ds / r_turn_min
            xl += gear * ds * umath.cos(thl)
            yl += gear * ds * umath.sin(thl)
            gx = X0 + xl * cos0 - yl * sin0
            gy = Y0 + xl * sin0 + yl * cos0
            parts.append("{:.1f},{:.1f}".format(gx, gy))
            if len(parts) >= batch:
                print("PLAN " + ";".join(parts))
                parts = []
    if parts:
        print("PLAN " + ";".join(parts))


# ----------------------------------------------------------------------------
# Control del vehiculo a lo largo de un tramo (leg)
# ----------------------------------------------------------------------------
def controlDelVehiculo(path, leg_start, seg_offset, seg_total, h=1):
    """Maneja el auto por un tramo. Devuelve 'ok', 'stop', 'stall' o
    'timeout'."""
    elapsed_time_max = 30      # [seg] watchdog por tramo
    elapsed_time = 0
    resetAllSensors()

    steering_reference = 0.0
    delta_s = h * v / 1000.0

    # Frame global del tramo, para reportar la posicion (x, y) del auto.
    X0 = leg_start[0]
    Y0 = leg_start[1]
    TH0 = umath.radians(leg_start[2])
    cos0 = umath.cos(TH0)
    sin0 = umath.sin(TH0)
    x_local = 0.0
    y_local = 0.0

    for i in range(len(path)):
        print("SEG {} {}".format(seg_offset + i + 1, seg_total))

        distance = path[i]['distance']
        steer_dir = path[i]['steering']
        gear = path[i]['gear']

        front.reset_angle(0)
        rear.reset_angle(0)
        position_reference = 0.0
        position_vehicle = 0.0
        position_vehicle_prev = 0.0
        steering_I = 0.0
        position_I = 0.0

        while abs(position_vehicle) <= distance and (elapsed_time / 1000.0) < elapsed_time_max:
            # STOP de emergencia desde el dashboard
            if read_command() == "STOP":
                audi.drive_power(0)
                audi.steer(0)
                return 'stop'

            # Motores bloqueados
            if rear.stalled() or front.stalled():
                audi.drive_power(0)
                audi.steer(0)
                return 'stall'

            # Referencias: posicion y rumbo avanzan juntos y solo mientras la
            # referencia no llego al final del tramo.
            if abs(position_reference) < abs(distance):
                position_reference += gear * delta_s
                steering_reference += gear * steer_dir * delta_s / r_turn_min

            # Estado del vehiculo
            position_vehicle = 0.5 * (front.angle() + rear.angle()) * ANGLE_TO_CM
            steering_vehicle = hub.imu.heading()

            # Errores
            position_error = position_reference - position_vehicle
            steering_error = umath.degrees(steering_reference) - steering_vehicle

            # Acciones de control (PI)
            position_P = GAINS['kp_pos'] * position_error
            position_I += GAINS['ki_pos'] * position_error
            steering_P = GAINS['kp_st'] * steering_error
            steering_I += GAINS['ki_st'] * steering_error

            position_command = int(min(max(position_P + position_I, -max_drive_power), max_drive_power))
            steering_command = gear * int(min(max(steering_P + steering_I, -max_angle_power), max_angle_power))

            audi.steer(steering_command)
            audi.drive_power(position_command)

            # Integracion de la posicion (x, y) global del auto
            ds = position_vehicle - position_vehicle_prev
            position_vehicle_prev = position_vehicle
            heading_rad = umath.radians(steering_vehicle)
            x_local += ds * umath.cos(heading_rad)
            y_local += ds * umath.sin(heading_rad)

            wait(h)
            elapsed_time += h

            # Telemetria al dashboard, cada 100 ms
            if elapsed_time % 100 == 0:
                x_global = X0 + x_local * cos0 - y_local * sin0
                y_global = Y0 + x_local * sin0 + y_local * cos0
                print("T {:.2f},{:.2f},{:.2f},{:.2f},{},{:.2f},{:.2f},{:.2f},{},{:.1f},{:.1f}".format(
                    run_watch.time() / 1000.0,
                    position_reference, position_vehicle, position_error, position_command,
                    umath.degrees(steering_reference), steering_vehicle, steering_error, steering_command,
                    x_global, y_global))

        # Watchdog de tiempo
        if (elapsed_time / 1000.0) >= elapsed_time_max:
            audi.drive_power(0)
            audi.steer(0)
            return 'timeout'

    audi.drive_power(0)
    audi.steer(0)
    return 'ok'


# ----------------------------------------------------------------------------
# Ejecucion de la trayectoria completa
# ----------------------------------------------------------------------------
def run_trajectory(PATH):
    run_watch.reset()
    hub.light.on(Color.ORANGE)

    # Calcular la trayectoria optima de Reeds-Shepp de cada tramo
    legs = []
    seg_total = 0
    for i in range(len(PATH) - 1):
        start_point, end_point = normalize_start_and_end_point(PATH[i], PATH[i + 1], r_turn_min)
        best = get_best_path(start_point, end_point)
        if best is None:
            print("ERR sin solucion Reeds-Shepp en el tramo {}".format(i + 1))
            print("END error")
            hub.light.on(Color.RED)
            return
        leg = denormalize_distance_in_path(best, r_turn_min)
        legs.append(leg)
        seg_total += len(leg)
        gc.collect()

    # Mandar la trayectoria planificada al dashboard, tramo por tramo
    for i in range(len(legs)):
        gc.collect()
        stream_plan(legs[i], PATH[i])

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
    setup()
    PATH = None
    print("RDY")
    print("VER 7 rs-stream")

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
                print("ACK PATH {}".format(len(PATH)))
            else:
                PATH = None
                print("ERR PATH invalido (minimo 2 poses x,y,h)")

        elif line.startswith("PID "):
            if set_gains(line[4:]):
                print("ACK PID {},{},{},{}".format(
                    GAINS['kp_pos'], GAINS['ki_pos'], GAINS['kp_st'], GAINS['ki_st']))
            else:
                print("ERR PID invalido (4 numeros)")

        elif line == "START":
            if not PATH:
                print("ERR sin PATH cargado")
            else:
                print("ACK START")
                run_trajectory(PATH)
                print("RDY")

        elif line == "STOP":
            audi.drive_power(0)
            audi.steer(0)
            print("ACK STOP")

        elif line == "PING":
            print("RDY")

        else:
            print("ERR comando desconocido: " + line)


if __name__ == "__main__":
    main()
