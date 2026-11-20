# =============================================================================
# hardware.py - EL AUTO: hub, motores y sensores
# =============================================================================
# Unico modulo que toca los motores y el IMU (junto con main_rs.py que usa el
# reloj). Si cambia el cableado o se cambia el auto, se cambia solo aca.
#
#   Puerto D  motor de direccion      Puertos A y B  motores de traccion (trasero y delantero)
#
# El resto del codigo trabaja con magnitudes fisicas: read_position() devuelve cm,
# read_heading() grados, y los comandos son porcentajes de potencia.
# =============================================================================
from pybricks.hubs import TechnicHub
from pybricks.parameters import Color, Direction, Port
from pybricks.tools import StopWatch
from pybricks.robotics import Car
from pybricks.pupdevices import Motor

ANGLE_TO_CM = 0.045     # [cm por grado de motor] (rueda, ver vehicleConstants)

hub = TechnicHub()
steering = Motor(Port.D, Direction.CLOCKWISE)
front = Motor(Port.B, Direction.CLOCKWISE)
rear = Motor(Port.A, Direction.CLOCKWISE)
audi = Car(steering, [front, rear])

run_watch = StopWatch()     # reloj del ensayo [ms]


def setup():
    """Inicializacion del hub: luz roja (no listo) y filtro del IMU."""
    hub.light.on(Color.RED)
    hub.imu.settings(1.5, 250)


def stop_car():
    """Frena la traccion y centra la direccion. Se llama desde STOP, stall,
    timeout y fin de tramo."""
    audi.drive_power(0)
    audi.steer(0)


def reset_sensors():
    """Frena el auto y pone a cero IMU y encoders (marco del tramo)."""
    audi.drive_power(0)
    audi.steer(0)
    hub.imu.reset_heading(0)
    front.reset_angle(0)
    rear.reset_angle(0)


def reset_encoders():
    """Encoders a cero (se hace al empezar cada segmento)."""
    front.reset_angle(0)
    rear.reset_angle(0)


def read_position():
    """Arco recorrido desde el ultimo reset de encoders [cm] (promedio de los dos)."""
    return 0.5 * (front.angle() + rear.angle()) * ANGLE_TO_CM


def read_heading():
    """Rumbo del IMU [grados], convencion compass (CW positivo)."""
    return hub.imu.heading()


def motors_stalled():
    """True si algun motor de traccion esta bloqueado."""
    return rear.stalled() or front.stalled()


def battery():
    """(tension [mV], corriente de descarga [mA]); (0, 0) si no se puede leer."""
    try:
        return hub.battery.voltage(), hub.battery.current()
    except:
        return 0, 0
