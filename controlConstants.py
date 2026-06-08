# controlConstants.py

# Ganancias de control Proporcional-Integral (PI).
# Son la semilla inicial de GAINS en main_rs.py. El dashboard pisa estos
# valores con el comando PID en cada START, asi que solo se usan al bootear
# el hub o si se corre el firmware sin dashboard.
KP_POSITION = 50.0
KI_POSITION = 0.01

KP_STEERING = 50.0
KI_STEERING = 0.01
