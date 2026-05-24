# =============================================================================
# SOLUCION DE PROBLEMAS - Conexion Bluetooth (BLE) con el hub
# =============================================================================
# Sintoma: pybricksdev o la extension de VSCode fallan al conectar, con un
#   error tipo "Could not find all requested services" o
#   "BleakCharacteristicNotFoundError: Characteristic ... was not found".
#   Pybricks Code (en el navegador) conecta bien, pero pybricksdev no.
#
# Solucion (la que funciono):
#   1. Cerrar todo lo conectado al hub: Pybricks Code y las pestañas de
#      Chrome. Una sola conexion BLE a la vez.
#   2. Apagar y prender el hub.
#   3. Administrador de dispositivos -> menu Ver -> "Mostrar dispositivos
#      ocultos" -> seccion Bluetooth -> buscar el hub ("Obi Juan's Hub" o
#      similar) -> clic derecho -> Desinstalar el dispositivo.
#   4. Volver a correr pybricksdev. Windows redescubre la tabla GATT de cero.
# =============================================================================

from pybricks.hubs import TechnicHub
from pybricks.parameters import Color, Direction, Port, Stop
from pybricks.tools import wait
from pybricks.robotics import Car
from pybricks.pupdevices import Motor

from reeds_shepp import get_all_paths, get_optimal_path, normalize_start_and_end_point, denormalize_distance_in_path
from vehicleConstants import L_car, psi_max, v, max_angle ,max_angle_power, max_drive_power, r_turn_min
from controlConstants import KP_POSITION, KI_POSITION, KP_STEERING, KI_STEERING
from debugFunctions import infoControl
import umath


CM_TO_ANGLE = 22 # 3600 grados / 162cm
ANGLE_TO_CM = 0.045


hub = TechnicHub()
steering = Motor(Port.D, Direction.CLOCKWISE)
front = Motor(Port.B, Direction.CLOCKWISE)
rear = Motor(Port.A, Direction.CLOCKWISE)
audi = Car(steering, [front, rear])


def resetAllSensors():
    audi.steer(0)
    hub.imu.reset_heading(0)
    front.reset_angle(0)
    rear.reset_angle(0)

def controlDelVehiculo(path, show_debug_info=False, h=1):
    elapsed_time_max = 30  # [seg] presupuesto TOTAL del recorrido (watchdog)
    elapsed_time = 0
    resetAllSensors()  # Asegúrate de que esta función esté definida
    
    steering_reference = 0.0
    delta_s = h * v / 1000.0  # `v` debe estar definida globalmente

    for i in range(len(path)):
        print("Tramo: ", i + 1)
        # obtener los valores del tramo:
        distance = path[i]['distance']
        steer_dir = path[i]['steering']   # -1 LEFT / 0 STRAIGHT / 1 RIGHT (no confundir con el Motor 'steering')
        gear = path[i]['gear']
                    
        # Reinicio de las variables
        front.reset_angle(0)  # `front` debe ser un objeto previamente definido
        rear.reset_angle(0)   # `rear` debe ser un objeto previamente definido
        position_reference = 0.0
        position_vehicle = 0.0
        steering_I = 0.0
        position_I = 0.0

        while abs(position_vehicle) <= distance and (elapsed_time / 1000.0) < elapsed_time_max:
            # Verificar que los motores no se han bloqueado
            if rear.stalled() or front.stalled():
                print("Motor bloqueado. Terminando")
                audi.drive_power(0)  # `audi` debe ser un objeto previamente definido
                audi.steer(0)
                return
            
            # Generar los nuevos valores de referencia.
            # Posicion y rumbo parametrizan el mismo arco por longitud de arco:
            # ambos avanzan juntos y SOLO mientras la referencia no llego al final
            # del tramo. Si no, el rumbo de referencia se pasa de largo del tramo.
            if abs(position_reference) < abs(distance):
                position_reference += gear * delta_s
                steering_reference += gear * steer_dir * delta_s / r_turn_min
            # Adquirir el estado actual del vehículo
            position_vehicle = 0.5 * (front.angle() + rear.angle()) * ANGLE_TO_CM
            steering_vehicle = hub.imu.heading()  # `hub` debe ser un objeto previamente definido

            # Calcular los errores
            position_error = position_reference - position_vehicle
            steering_error = umath.degrees(steering_reference) - steering_vehicle

            # Calcular las acciones de control
            position_P = KP_POSITION * position_error
            position_I += KI_POSITION * position_error
            steering_P = KP_STEERING * steering_error
            steering_I += KI_STEERING * steering_error

            position_command = int(min(max(position_P + position_I, -max_drive_power), max_drive_power))
            
            steering_command = gear * int(min(max(steering_P + steering_I, -max_angle_power), max_angle_power))

            # Ejecutar las acciones de control
            audi.steer(steering_command)
            audi.drive_power(position_command)
            
            wait(h)  # Asegúrate de que `wait()` esté definida
            elapsed_time += h

            # Imprimir los valores actuales
            if elapsed_time % 100 == 0 and show_debug_info:
                infoControl(elapsed_time / 1000.0, position_reference, position_vehicle, position_error, position_command,
                            steering_reference, steering_vehicle, steering_error, steering_command)
        
        #imprimir la última información
        infoControl(elapsed_time / 1000.0, position_reference, position_vehicle, position_error, position_command,
                            steering_reference, steering_vehicle, steering_error, steering_command)
        
        # Watchdog: si se agoto el presupuesto de tiempo sin completar el recorrido, abortar.
        if (elapsed_time / 1000.0) >= elapsed_time_max:
            print("Timeout: {} seg sin completar el recorrido. Terminando.".format(elapsed_time_max))
            audi.drive_power(0)
            audi.steer(0)
            return

        #print("Llegó al final del tramo {}".format(i + 1))
    # Detener el vehículo y la dirección después de completar el recorrido
    audi.drive_power(0)
    audi.steer(0)
    hub.light.on(Color.GREEN)  # Usa una cadena si `Color.RED` no está definido
    wait(2000)
    print("Conducción completada en {:.2f} seg.".format(elapsed_time / 1000.0))

 

def setup():

   # global steering_max_right, steering_max_left  # ← Agregar esto

    hub.light.on(Color.RED)
    hub.imu.settings(1.5, 250)

    

def test_control():
    # originalmente definido en reeds_shepp.py
    FORWARD = 1
    BACKWARD = -1
    LEFT = -1
    RIGHT = 1
    STRAIGHT = 0
    path = [
        {'distance': 40.0, 'steering': STRAIGHT, 'gear': FORWARD},  # Recto, avance
        {'distance': 40.0, 'steering': STRAIGHT, 'gear': BACKWARD},  # Recto, retroceso
        {'distance': 40.0, 'steering': RIGHT, 'gear': FORWARD},  # Giro a la derecha, avance
        {'distance': 40.0, 'steering': RIGHT, 'gear': BACKWARD},  # Giro a la derecha, retroceso
        {'distance': 40.0, 'steering': LEFT, 'gear': FORWARD},  # Giro a la izquierda, avance
        {'distance': 40.0, 'steering': LEFT, 'gear': BACKWARD},  # Giro a la izquierda, retroceso
        
    ]
    controlDelVehiculo(path, show_debug_info=True)
    
def trayectory_control():
    p1 = [0.0, 0.0, 0.0]  # Postura inicial
    p2 = [0.0, 0.0, 90.0]  # Postura final
    p3 = [0.0, 0.0, 180.0]
    p4 = [0.0, 0.0, 180.0]
    PATH = [p1, p2, p3]#, p3, p4]
    
    for i in range(len(PATH) - 1):

        start_point_original = PATH[i]
        end_point_original = PATH[i + 1]
        # Normalizar el punto final
        start_point, end_point = normalize_start_and_end_point(start_point_original, end_point_original, r_turn_min)
        # Obtener los caminos
        paths = get_all_paths(start_point, end_point)

        # Obtener el índice del camino óptimo
        min_distance_index = get_optimal_path(paths)

        # Obtener el camino óptimo
        path = paths[min_distance_index]

        # Desnormalizar el camino
        path = denormalize_distance_in_path(path, r_turn_min)
        
        #infoTrayectoria(init_pos, final_pos, L, path_type, estimated_driving_time)
        print("Iniciando control trayectoria")
        print(path)
        controlDelVehiculo(path, True)
        
        
    print("Control finalizado")


if __name__ == "__main__":
    print("Si el programa no carga cerrar VSCode, apagar el Hub, y comenzar de nuevo.")
    print("Presionar F5 para cargarlo")
    print("Para no complicarse: https://code.pybricks.com/")
    print("Voltage: ", hub.battery.voltage() / 1000, "IMU Ready:", hub.imu.ready(), "IMU Stationary", hub.imu.stationary())
    
    setup()
    #test_control()
    trayectory_control()
           
    # Asegúrate de limpiar recursos aquí si es necesario
    print("Finalizando el programa correctamente.")
    hub.light.on(Color.GREEN)
    wait(3000)


