# AUTO LEGO: arquitectura del firmware (VER 16, modular)

Cada archivo hace una sola cosa. main_rs.py solo los orquesta.

## Mapa de módulos

| Archivo | Qué hace | ¿Toca hardware? |
|---|---|---|
| settings.py | Parámetros ajustables en caliente (objeto cfg) y parsers de los comandos del dashboard (VEL, PROF, ACC, PI, ALG, CTRL, SMC, PATH) | no |
| trajectory.py | Generación de trayectoria: planner Reeds-Shepp / Dubins, armado de tramos, filtro de segmentos nulos, envío del plan al dashboard | no |
| velocity_profile.py | Rampas: perfil de velocidad constante o trapezoidal de la referencia | no |
| reference.py | La referencia que recorre el camino: arco, rumbo y pose (x, y) que el auto debe perseguir | no |
| odometry.py | Dónde está el auto (x, y) y errores e_s (atraso) y e_y (lateral) respecto de la referencia | no |
| controllers.py | Lazos de control: una clase PI (con antiwindup) usada para tracción y para dirección con distintas ganancias, SMC de dirección, anticipación de curvatura | no |
| smc_control.py | La matemática del modo deslizante (LateralSMC) | no |
| hardware.py | Hub, motores, IMU, encoders, batería | sí |
| comms.py | Entrada de comandos (stdin) y salida de telemetría (print) | sí (stdin) |
| main_rs.py | Lazo principal a periodo fijo y dispatcher de comandos | orquesta |

## Un tick del lazo (cada LOOP_MS = 5 ms)

perfil de velocidad, referencia, sensores, errores, controladores, motores, telemetría (cada 100 ms).

1. La referencia avanza: el perfil dice cuántos cm (ds) y si es el último tick del segmento; Reference integra arco, rumbo y pose.
2. Se leen encoders e IMU (hardware.py).
3. Errores: position_error = arco_ref - arco_auto; steering_error = rumbo_ref - rumbo_IMU; e_s y e_y desde la odometría.
4. Controladores: un PI da la potencia de tracción; SteeringPI (el mismo PI con otras ganancias, más el signo de la marcha) o SteeringSMC da la de dirección. Todos devuelven (comando, parte_P, parte_I). Los PI tienen antiwindup: integración condicional (no integran si el comando ya está saturado y el incremento lo empuja más afuera) y tope duro de la integral.
5. Se aplican a los motores y se espera hasta el próximo tick (periodo fijo; los ticks pasados de tiempo se recuperan).

## Cómo se agrega algo

- Un perfil de velocidad nuevo (por ejemplo curva en S): una clase con método next() que devuelva (ds, last) en velocity_profile.py, y una rama en make_profile().
- Un controlador de dirección nuevo: una clase en controllers.py con update(gear, steering_error, e_y, e_s, ds, kappa_ref, k_lead) que devuelva (comando, P, I) y reset(); registrarla en make_steering_controller() y aceptar su nombre en set_controller().
- Un comando nuevo del dashboard: set_xxx() en settings.py y una rama en main() de main_rs.py.

## Reglas para que el bundle funcione

El hub necesita un solo archivo: `python SMC/make_bundle.py` genera main_bundle.py juntando todos los módulos (y sacándoles comentarios y docstrings para ahorrar RAM al compilar). En el bundle todos los módulos comparten un solo namespace, por eso:
- usar `from modulo import nombre` (nunca `import modulo` y `modulo.nombre`);
- no repetir nombres globales entre módulos;
- volver a correr make_bundle.py después de editar cualquier .py del hub.

Orden de dependencias en make_bundle.py: vehicleConstants, controlConstants, reeds_shepp, dubinsPathPlanning, smc_control, settings, velocity_profile, reference, odometry, hardware, comms, trajectory, controllers, main_rs.

## Tests

- `python3 SMC/test_modulos.py`: módulos puros (perfiles, referencia, odometría, parsers). También sirve de ejemplo de uso.
- `PYTHONPATH=. python3 SMC/test_firmware_stub.py`: firmware completo con planta simulada (VEL=40 para cambiar la velocidad; FW_MODULE=main_bundle prueba el archivo único).
- `node SMC/test_dashboard.js` (necesita jsdom): dashboard.

La reorganización se verificó comparando la telemetría tick a tick del firmware antes y después (PI y SMC, a 17 y 40 cm/s, y con perfil constante): idéntica byte a byte, con los módulos sueltos y con el bundle comprimido. Bundle: 43 KB (el anterior pesaba 59 KB).

## Perfil de velocidad (resumen)

PROF TRAP (default) o CONST; ACC en cm/s² (default 80). El trapecio acelera y frena solo al inicio del recorrido, al final y en los cambios de marcha; entre tramos del mismo sentido mantiene la velocidad. A 40 cm/s con perfil constante el auto cruza el final a ~40 cm/s y avanza ~6 cm por inercia; con trapecio a=50 cruza a ~6 cm/s y avanza ~1 cm.
