# =============================================================================
# test_modulos.py - Pruebas de los modulos que NO dependen del hardware.
#   python3 SMC/test_modulos.py
# Sirve tambien como ejemplo de como se usa cada modulo por separado.
# =============================================================================
import sys, os, math
REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO)
sys.modules['umath'] = math

from velocity_profile import make_profile, ConstantProfile
from reference import Reference
from odometry import Odometry
from settings import cfg, parse_path, set_velocity, set_profile, set_accel, set_gains

ok = True
def check(name, cond):
    global ok
    ok &= bool(cond)
    print(('OK    ' if cond else 'FALLA ') + name)

def run(p):
    s, vmax, vend = 0.0, 0.0, 0.0
    while True:
        ds, last = p.next()
        s += ds
        vmax = max(vmax, ds / 0.005); vend = ds / 0.005
        if last: return s, vmax, vend

# --- perfiles de velocidad ---
s, vmax, vend = run(ConstantProfile(100.0, 40.0, 5))
check('constante: recorre exactamente la distancia (%.3f cm)' % s, abs(s - 100.0) < 1e-6)
check('constante: velocidad constante (%.1f cm/s)' % vmax, abs(vmax - 40.0) < 2.0)

s, vmax, vend = run(make_profile('TRAP', 100.0, 1, None, None, 40.0, 80.0, 5))
check('trapezoidal: recorre exactamente la distancia (%.3f cm)' % s, abs(s - 100.0) < 1e-6)
check('trapezoidal: llega a la velocidad maxima (%.1f cm/s)' % vmax, abs(vmax - 40.0) < 1.0)
check('trapezoidal: termina casi en reposo (%.1f cm/s)' % vend, vend < 10.0)

s, vmax, vend = run(make_profile('TRAP', 5.0, 1, None, None, 40.0, 80.0, 5))
check('trapezoidal: tramo corto = triangular, no llega a v_max (pico %.1f cm/s)' % vmax,
      vmax < 25.0 and abs(s - 5.0) < 1e-6)

# entre tramos del mismo sentido NO frena; en un cambio de marcha SI
s, vmax, vend = run(make_profile('TRAP', 60.0, 1, 1, 1, 40.0, 80.0, 5))
check('mismo sentido: no frena entre tramos (v final %.1f cm/s)' % vend, vend > 35.0)
s, vmax, vend = run(make_profile('TRAP', 60.0, 1, 1, -1, 40.0, 80.0, 5))
check('cambio de marcha: frena al final (v final %.1f cm/s)' % vend, vend < 10.0)

# --- referencia: recta de 100 cm hacia adelante ---
ref = Reference()
ref.start_segment(100.0, 0, 1, make_profile('TRAP', 100.0, 1, None, None, 40.0, 80.0, 5))
while not ref.done:
    ref.advance()
check('referencia: arco final = 100 cm', abs(ref.position - 100.0) < 1e-9)
check('referencia: recta no gira el rumbo', abs(ref.heading) < 1e-12)
check('referencia: x final ~ 100 cm, y ~ 0', abs(ref.x - 100.0) < 0.05 and abs(ref.y) < 1e-9)

# --- odometria: auto 2 cm a la izquierda de la traza ---
odo = Odometry()
odo.x, odo.y = 50.0, 2.0
r2 = Reference(); r2.x, r2.y = 50.0, 0.0
e_s, e_y = odo.tracking_errors(r2)
check('odometria: e_y = +2 cm a la izquierda (%.2f)' % e_y, abs(e_y - 2.0) < 1e-9 and abs(e_s) < 1e-9)

# --- comandos / settings ---
check('parse_path', parse_path('0,0,0;100,50,-90') == [[0, 0, 0], [100, 50, -90]])
check('parse_path mal formado -> None', parse_path('0,0;1,2,3') is None)
check('VEL valido', set_velocity('40') and cfg.velocity == 40.0)
check('VEL invalido', (not set_velocity('-3')) and cfg.velocity == 40.0)
check('PROF', set_profile('const') and cfg.profile == 'CONST' and not set_profile('xxx'))
check('ACC', set_accel('60') and cfg.accel == 60.0 and not set_accel('-1'))
check('PI', set_gains('50,0.01,25,0.02') and cfg.gains['kp_st'] == 25.0 and not set_gains('1,2,3'))

# --- antiwindup de los PI (controllers.integrate) ---
# controllers importa pybricks-free modulos (settings, smc_control): se puede probar en la PC
from controllers import integrate
i = 0.0
for _ in range(10000):                      # error constante enorme: la integral no debe pasar del tope
    i = integrate(i, 50.0, 1.0, 100)
check('antiwindup: integral acotada a +-100 (%.1f)' % i, abs(i) <= 100.0)
i = 0.0
for _ in range(1000):                       # comando ya saturado por la parte P: no integra
    i = integrate(i, 150.0, 1.0, 100)
check('antiwindup: no integra con el comando saturado (%.1f)' % i, i == 0.0)
i = 30.0
i2 = integrate(i, 150.0, -1.0, 100)         # el incremento descarga: debe poder integrar
check('antiwindup: si el incremento descarga, integra (%.1f -> %.1f)' % (i, i2), i2 == 29.0)

print('PRUEBA', 'OK' if ok else 'FALLA')
sys.exit(0 if ok else 1)
