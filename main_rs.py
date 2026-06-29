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
#   ALG RS|DUBINS                   elige el planner (default RS)
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
#   T t,pr,pv,pe,pc,sr,sv,se,sc,x,y,pP,pI,sP,sI   telemetria (ver mas abajo)
#   END ok|stop|stall|timeout|error   fin del ensayo
#   BAT mv,ma                      bateria: tension [mV] y corriente [mA]
#
# Campos de la telemetria T:
#   t  tiempo [s]            pr posicion referencia [cm]   pv posicion real [cm]
#   pe error posicion [cm]   pc comando traccion           sr rumbo ref [grados]
#   sv rumbo real [grados]   se error rumbo [grados]       sc comando direccion
#   x  posicion X global [cm]   y posicion Y global [cm]
#   pP/pI accion traccion P e I    sP/sI accion direccion P e I (con gear)
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

from pybricks.hubs import TechnicHub
from pybricks.parameters import Color, Direction, Port
from pybricks.tools import wait, StopWatch
from pybricks.robotics import Car
from pybricks.pupdevices import Motor

from reeds_shepp import get_best_path, normalize_start_and_end_point, denormalize_distance_in_path
from dubinsPathPlanning import get_best_dubins
from vehicleConstants import v, max_angle_power, max_drive_power, r_turn_min
from controlConstants import KP_POSITION, KI_POSITION, KP_STEERING, KI_STEERING
import umath
import gc

# Modulos estandar de MicroPython para leer stdin. Este es el metodo oficial
# de Pybricks para recibir datos por BLE (ver tutorial "Hub to PC Communication").
from usys import stdin
from uselect import poll


ANGLE_TO_CM = 0.045

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

# Ganancias de los PID. Arrancan con los valores de controlConstants.py; el
# comando PID las cambia entre ensayos.
GAINS = {'kp_pos': KP_POSITION, 'ki_pos': KI_POSITION,
         'kp_st': KP_STEERING,  'ki_st': KI_STEERING}

# Algoritmo de planificacion. 'RS' o 'DUBINS'. El comando ALG lo cambia.
ALGORITHM = 'RS'


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


def set_algorithm(arg):
    """'RS' o 'DUBINS' -> actualiza ALGORITHM. Devuelve True si ok."""
    global ALGORITHM
    arg = arg.strip().upper()
    if arg not in ('RS', 'DUBINS'):
        return False
    ALGORITHM = arg
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
                   x, y, pP, pI, sP, sI):
    """Imprime una linea T de telemetria al dashboard (formato y campos en
    la cabecera del archivo)."""
    print("T {:.2f},{:.2f},{:.2f},{:.2f},{},{:.2f},{:.2f},{:.2f},{},{:.1f},{:.1f},{:.1f},{:.1f},{:.1f},{:.1f}".format(
        t, pos_ref, pos_veh, pos_err, pos_cmd,
        st_ref_deg, st_veh, st_err, st_cmd,
        x, y, pP, pI, sP, sI))


def controlDelVehiculo(path, leg_start, seg_offset, seg_total, h=1):
    """Maneja el auto por un tramo. Devuelve 'ok', 'stop', 'stall' o
    'timeout'."""
    elapsed_time_max = 30      # [seg] watchdog por tramo
    elapsed_time = 0
    resetAllSensors()

    steering_reference = 0.0
    delta_s = h * v / 1000.0

    # Frame global del tramo, para reportar la posicion (x, y) del auto.
    to_global = make_frame(leg_start)
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
                stop_car()
                return 'stop'

            # Motores bloqueados
            if rear.stalled() or front.stalled():
                stop_car()
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

            # --- Accion de control de POSICION (traccion), PI ---
            position_P = GAINS['kp_pos'] * position_error
            position_I += GAINS['ki_pos'] * position_error
            position_command = saturate(position_P + position_I, max_drive_power)

            # --- Accion de control de DIRECCION (steering), PI ---
            steering_P = GAINS['kp_st'] * steering_error
            steering_I += GAINS['ki_st'] * steering_error
            steering_command = gear * saturate(steering_P + steering_I, max_angle_power)

            audi.steer(steering_command)
            audi.drive_power(position_command)

            # Integracion de la posicion (x, y) global del auto. El rumbo del
            # IMU se invierte de signo SOLO para esta integracion (convencion
            # matematica estandar), no afecta el lazo de control.
            ds = position_vehicle - position_vehicle_prev
            position_vehicle_prev = position_vehicle
            heading_rad = -umath.radians(steering_vehicle)
            x_local += ds * umath.cos(heading_rad)
            y_local += ds * umath.sin(heading_rad)

            wait(h)
            elapsed_time += h

            # Telemetria al dashboard, cada 100 ms
            if elapsed_time % 100 == 0:
                x_global, y_global = to_global(x_local, y_local)
                send_telemetry(
                    run_watch.time() / 1000.0,
                    position_reference, position_vehicle, position_error, position_command,
                    umath.degrees(steering_reference), steering_vehicle, steering_error, steering_command,
                    x_global, y_global,
                    position_P, position_I, gear * steering_P, gear * steering_I)

        # Watchdog de tiempo
        if (elapsed_time / 1000.0) >= elapsed_time_max:
            stop_car()
            return 'timeout'

    stop_car()
    return 'ok'


# ----------------------------------------------------------------------------
# Ejecucion de la trayectoria completa
# ----------------------------------------------------------------------------
def run_trajectory(PATH):
    run_watch.reset()
    hub.light.on(Color.ORANGE)

    # Calcular la trayectoria optima de cada tramo segun el algoritmo elegido
    legs = []
    seg_total = 0
    for i in range(len(PATH) - 1):
        leg = plan_leg(PATH[i], PATH[i + 1])
        if leg is None:
            print("ERR sin solucion {} en el tramo {}".format(ALGORITHM, i + 1))
            print("END error")
            hub.light.on(Color.RED)
            return
        leg = [seg for seg in leg if seg['distance'] >= MIN_SEG_DISTANCE]
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
    print("VER 11 rs-dubins-pi-split")
    send_battery()

    while True:
        line = read_command()
        if line is None:
            wait(20)
            continue
        if line == "":
            continue

        print("DBG len={} <{}>".format(len(line), line))  # TEMPORAL: diagnostico PATH

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

        elif line.startswith("ALG "):
            if set_algorithm(line[4:]):
                print("ACK ALG {}".format(ALGORITHM))
            else:
                print("ERR ALG invalido (RS o DUBINS)")

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
