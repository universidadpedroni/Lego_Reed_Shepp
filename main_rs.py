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
#   ey error lateral [cm] (odometria, marco de la referencia)   ss reservado (siempre 0)
#   pc=clamp(pP+pI), sc=gear*clamp(sP+sI). Si pP+pI o sP+sI exceden el
#   limite, pc/sc saturan pero pP,pI,sP,sI siguen: asi se ve el windup.
# =============================================================================
#
# =============================================================================
# NOTA SOBRE CONVENCION DE SIGNO DEL RUMBO PARA LA VISUALIZACION
# =============================================================================
# El planner (reeds_shepp) usa LEFT=-1 y el IMU del hub
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

# =============================================================================
# MAPA DE MODULOS (cada archivo hace UNA cosa; main_rs.py solo los orquesta)
# =============================================================================
#   settings.py          parametros ajustables (cfg) y parsers de los comandos del dashboard
#   trajectory.py        generacion de trayectoria: planner Reeds-Shepp, tramos, PLAN al dashboard
#   velocity_profile.py  rampas: perfil de velocidad constante o trapezoidal de la referencia
#   reference.py         la referencia: arco, rumbo y pose que el auto debe perseguir
#   odometry.py          donde esta el auto (x, y) y errores e_s / e_y respecto de la referencia
#   controllers.py       lazos de control: PI de traccion y PI de direccion
#   hardware.py          hub, motores, IMU, encoders, bateria
#   comms.py             entrada de comandos (stdin) y salida de telemetria (print)
#   main_rs.py           lazo principal a periodo fijo + dispatcher de comandos (este archivo)
#
# Flujo de un tick del lazo (controlDelVehiculo):
#   perfil -> referencia -> sensores -> errores -> controladores -> motores -> telemetria
#
# Para subir al hub UN solo archivo: python SMC/make_bundle.py genera main_bundle.py
# juntando todos estos modulos. Hay que volver a correrlo despues de editar cualquiera.
# =============================================================================

import gc
import umath
from pybricks.parameters import Color
from pybricks.tools import wait

from settings import cfg, LOOP_MS, parse_path, set_gains, set_velocity, set_profile, set_accel
from hardware import (audi, hub, run_watch, setup, stop_car, reset_sensors, reset_encoders,
                      read_position, read_heading, motors_stalled)
from comms import read_command, send_battery, send_telemetry
from trajectory import build_legs, send_plan
from velocity_profile import make_profile
from reference import Reference
from odometry import Odometry, make_frame
from controllers import make_position_controller, make_steering_controller


# ----------------------------------------------------------------------------
# Control del vehiculo a lo largo de un tramo (leg)
# ----------------------------------------------------------------------------
def controlDelVehiculo(path, leg_start, seg_offset, seg_total, h=LOOP_MS):
    """Maneja el auto por un tramo (una lista de segmentos). Devuelve 'ok', 'stop',
    'stall' o 'timeout'.

    Un TICK del lazo (cada h ms), en orden:
      1. la referencia avanza (perfil de velocidad)      -> reference.py
      2. se leen los sensores                            -> hardware.py
      3. se calculan los errores                         -> odometry.py
      4. los controladores deciden las potencias         -> controllers.py
      5. se aplican a los motores y se espera al proximo tick
      6. cada 100 ms se manda telemetria                 -> comms.py"""
    elapsed_time_max = 30      # [seg] watchdog por tramo
    elapsed_time = 0
    reset_sensors()

    ref = Reference()                                   # referencia (arco, rumbo, x, y)
    odo = Odometry()                                    # odometria del auto
    pos_ctrl = make_position_controller(h)               # traccion (PI)
    steer_ctrl = make_steering_controller(h)             # direccion (PI)
    to_global = make_frame(leg_start)                   # marco global, para el dashboard

    # Lazo a periodo fijo: next_tick es el instante en que debe arrancar el proximo tick.
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

        # Perfil de velocidad del segmento y arranque de la referencia.
        profile = make_profile(cfg.profile, distance, gear, prev_gear, next_gear,
                               cfg.velocity, cfg.accel, h)
        ref.start_segment(distance, steer_dir, gear, profile)

        reset_encoders()
        odo.reset_segment()
        position_vehicle = 0.0
        pos_ctrl.reset()
        steer_ctrl.reset()

        while abs(position_vehicle) <= distance and (elapsed_time / 1000.0) < elapsed_time_max:
            # STOP de emergencia desde el dashboard
            if read_command() == "STOP":
                stop_car()
                return 'stop'

            # Motores bloqueados
            if motors_stalled():
                stop_car()
                return 'stall'

            # 1. Referencia
            ref.advance()

            # 2. Estado del vehiculo
            position_vehicle = read_position()
            steering_vehicle = read_heading()

            # 3. Errores
            position_error = ref.position - position_vehicle
            steering_error = umath.degrees(ref.heading) - steering_vehicle
            odo.update(position_vehicle, steering_vehicle)
            e_s, e_y = odo.tracking_errors(ref)

            # 4. Control: traccion (PI de posicion) y direccion (PI de rumbo)
            position_command, position_P, position_I = pos_ctrl.update(position_error)
            steering_command, steering_P, steering_I = steer_ctrl.update(gear, steering_error)

            # 5. Motores
            audi.steer(steering_command)
            audi.drive_power(position_command)

            # Lazo a periodo fijo: espera lo que falta hasta el proximo tick.
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

            # 6. Telemetria al dashboard, cada 100 ms
            if elapsed_time % 100 == 0:
                x_global, y_global = to_global(odo.x, odo.y)
                send_telemetry(
                    run_watch.time() / 1000.0,
                    ref.position, position_vehicle, position_error, position_command,
                    umath.degrees(ref.heading), steering_vehicle, steering_error, steering_command,
                    x_global, y_global,
                    position_P, position_I, gear * steering_P, gear * steering_I,
                    e_y, steer_ctrl.s)

        # Watchdog de tiempo
        if (elapsed_time / 1000.0) >= elapsed_time_max:
            stop_car()
            return 'timeout'

    stop_car()
    if loop_ticks > 0:
        print("LOOP {:.2f} {}".format((run_watch.time() - loop_t0) / loop_ticks, overruns))
    return 'ok'



# ----------------------------------------------------------------------------
# Ejecucion de la trayectoria completa
# ----------------------------------------------------------------------------
def run_trajectory(PATH):
    run_watch.reset()
    hub.light.on(Color.ORANGE)

    # Reusar la trayectoria calculada al recibir PATH; si no hay, calcularla.
    if cfg.legs is None:
        cfg.legs, cfg.legs_total = build_legs(PATH)
        if cfg.legs is None:
            print("ERR sin solucion")
            print("END error")
            hub.light.on(Color.RED)
            return
        # Recalculado aca (no hubo preview al recibir PATH): mandarlo ahora.
        send_plan(cfg.legs, PATH)
    legs = cfg.legs
    seg_total = cfg.legs_total

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
# Comandos que solo cambian un parametro: prefijo -> funcion de settings.py que lo valida.
SETTERS = {'PI': set_gains, 'PROF': set_profile, 'ACC': set_accel, 'VEL': set_velocity}


def main():
    setup()
    PATH = None
    print("RDY")
    print("VER 17 RS+PI")
    send_battery()

    while True:
        line = read_command()
        if line is None:
            wait(20)
            continue
        if line == "":
            continue

        p = line.split(" ", 1)
        cmd = p[0]
        arg = p[1] if len(p) > 1 else ""

        if cmd == "PATH":
            poses = parse_path(arg)
            if poses and len(poses) >= 2:
                PATH = poses
                cfg.legs, cfg.legs_total = build_legs(PATH)
                if cfg.legs is None:
                    print("ERR sin solucion")
                else:
                    print("ACK PATH {}".format(len(PATH)))
                    send_plan(cfg.legs, PATH)
            else:
                PATH = None
                cfg.legs = None
                print("ERR PATH")

        elif cmd in SETTERS:
            # El hub devuelve el comando recibido: ACK <comando> o ERR <comando>.
            print(("ACK " if SETTERS[cmd](arg) else "ERR ") + line)

        elif cmd == "START":
            if not PATH:
                print("ERR sin PATH")
            else:
                print("ACK START")
                run_trajectory(PATH)
                send_battery()
                print("RDY")

        elif cmd == "STOP":
            stop_car()
            print("ACK STOP")

        elif cmd == "RESET":
            reset_sensors()
            send_battery()
            print("ACK RESET")

        elif cmd == "BAT":
            send_battery()

        elif cmd == "PING":
            print("RDY")

        else:
            print("ERR " + line)


if __name__ == "__main__":
    main()
