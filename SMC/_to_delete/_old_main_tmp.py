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
#   PROF CONST|TRAP                perfil de velocidad de la referencia (default TRAP)
#   ACC a                          aceleracion/frenado del perfil TRAP [cm/s^2]
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

from pybricks.hubs import TechnicHub
from pybricks.parameters import Color, Direction, Port
from pybricks.tools import wait, StopWatch
from pybricks.robotics import Car
from pybricks.pupdevices import Motor

from reeds_shepp import get_best_path, normalize_start_and_end_point, denormalize_distance_in_path
from dubinsPathPlanning import get_best_dubins
from vehicleConstants import v, max_angle_power, max_drive_power, r_turn_min, L_car, psi_max
from controlConstants import KP_POSITION, KI_POSITION, KP_STEERING, KI_STEERING
from smc_control import LateralSMC
import umath
import gc

# Modulos estandar de MicroPython para leer stdin. Este es el metodo oficial
# de Pybricks para recibir datos por BLE (ver tutorial "Hub to PC Communication").
from usys import stdin
from uselect import poll


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

# Perfil de velocidad TRAPEZOIDAL de la referencia. Con velocidad constante la referencia
# arrancaba y frenaba con un escalon: la traccion (tau ~0.15 s) no lo sigue y queda error al
# terminar. Ahora la referencia acelera con ACCEL [cm/s^2] hasta VELOCITY y frena con la misma
# rampa, SOLO al inicio del recorrido, al final y en los cambios de marcha (cusps); entre
# tramos del mismo sentido sigue a VELOCITY. PROFILE = 'CONST' (o ACCEL = 0) -> velocidad constante.
PROFILE = 'TRAP'  # 'CONST' (velocidad constante, escalon) | 'TRAP' (trapezoidal)
ACCEL = 80.0      # [cm/s^2] aceleracion/frenado del perfil TRAP
V_MIN_REF = 2.0   # [cm/s] piso de la rampa de frenado (para que la referencia llegue)

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


def set_profile(arg):
    """'CONST' o 'TRAP' -> actualiza PROFILE. Devuelve True si ok."""
    global PROFILE
    name = arg.strip().upper()
    if name != 'CONST' and name != 'TRAP':
        return False
    PROFILE = name
    return True


def set_accel(arg):
    """'<a>' en cm/s^2 -> actualiza ACCEL. Devuelve True si ok (numero >= 0)."""
    global ACCEL
    try:
        val = float(arg)
    except:
        return False
    if val < 0:
        return False
    ACCEL = val
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
        # Perfil trapezoidal: arranca/termina en reposo solo en los extremos del recorrido y en
        # los cambios de marcha; entre tramos del mismo sentido mantiene VELOCITY.
        v0_ref = VELOCITY if (i > 0 and path[i - 1]['gear'] == gear) else 0.0
        v1_ref = VELOCITY if (i + 1 < len(path) and path[i + 1]['gear'] == gear) else 0.0
        ref_v = v0_ref
        s_ref = 0.0
        ref_done = False

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
            if not ref_done:
                if PROFILE == 'TRAP' and ACCEL > 0:
                    # trapecio: sube con ACCEL, frena para llegar a v1_ref en 'distance'
                    rem = distance - s_ref
                    ref_v = min(VELOCITY, ref_v + ACCEL * h / 1000.0,
                                umath.sqrt(v1_ref * v1_ref + 2.0 * ACCEL * rem))
                    if ref_v < V_MIN_REF:
                        ref_v = V_MIN_REF
                    ds_ref = ref_v * h / 1000.0
                    last_tick = ds_ref >= rem
                    if last_tick:
                        ds_ref = rem
                    s_ref += ds_ref
                else:
                    ref_ticks += 1
                    ds_ref = ds_seg
                    last_tick = ref_ticks >= N_seg
                if not last_tick:
                    position_reference += gear * ds_ref
                    steering_reference += gear * steer_dir * ds_ref / r_turn_min
                else:
                    # ultimo tick del tramo: snap al valor exacto (mata la deriva
                    # por redondeo en float, equivalente al q = qf del resumen).
                    position_reference = gear * distance
                    steering_reference = head_ref_seg0 + gear * steer_dir * distance / r_turn_min
                    ref_done = True
                # pose de la referencia (convencion matematica, igual que la odometria)
                th_r = -steering_reference
                x_ref += gear * ds_ref * umath.cos(th_r)
                y_ref += gear * ds_ref * umath.sin(th_r)

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

        elif line.startswith("PROF "):
            if set_profile(line[5:]):
                print("ACK PROF {}".format(PROFILE))
            else:
                print("ERR PROF invalido (CONST o TRAP)")

        elif line.startswith("ACC "):
            if set_accel(line[4:]):
                print("ACK ACC {}".format(ACCEL))
            else:
                print("ERR ACC invalido (1 numero >= 0)")

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
