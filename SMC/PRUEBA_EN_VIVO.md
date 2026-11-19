# Prueba en vivo del SMC en el auto (reemplaza las secciones 8 y 9.5 de SMC_diseno.md)

Estado: SMC integrado en main_rs.py (VER 15) y en dashboard.html (v1.4). Verificado solo con stubs
(SMC/test_firmware_stub.py corre el controlDelVehiculo real contra la planta simulada; SMC/test_dashboard.js
prueba la logica del dashboard con jsdom). NO probado todavia en el hub real.

## Archivos que tienen que viajar al hub
main_rs.py, smc_control.py (NUEVO, en la raiz), reeds_shepp.py, dubinsPathPlanning.py, vehicleConstants.py,
controlConstants.py. Validacion: al conectar, el log del dashboard muestra "VER 15 ..." (version que muestra el hub al conectar).

## Protocolo nuevo (por BLE)
  CTRL PI|SMC                   elige el controlador de DIRECCION (default PI al bootear)
  SMC lam,eta,phi,ki,tlead,psi  parametros (defaults 0.035, 0.008, 0.05, 0, 0.2, 20)
  T ...,ey,ss                   17 campos: error lateral [cm] y variable de deslizamiento s [rad]
  LOOP ms ovr                   periodo real medio del lazo y cantidad de overruns al terminar cada tramo
El dashboard manda CTRL (y SMC si corresponde) antes de START. La traccion sigue siendo el PI de posicion.
Con SMC las barras de direccion muestran Eq (control equivalente) y Sw (conmutado) en vez de P e I.

## Procedimiento A/B (mismo PATH, mismas condiciones)
1. Bateria cargada, mismo piso, mismo punto de partida. Velocidad 40.
2. Selector en PI: 3 ensayos. Selector en Sliding mode: 3 ensayos. Alternar (PI, SMC, PI, SMC...) para que la
   bateria no sesgue la comparacion. El historial del panel de metricas guarda los ultimos 8.
3. Comparar: RMS/max lateral, RMS rumbo, actividad de direccion [%/s]. El lateral sale de odometria y se calcula
   igual con PI y con SMC.

## Criterios de aceptacion (del simulador, PATH [0,0,0] -> [120,60,-90])
- Lateral RMS: SMC < 60% del PI.
- Actividad de direccion: SMC < PI.
- Estado final "Completado" (sin stall/timeout) en los 3 ensayos.
- Lazo a periodo FIJO LOOP_MS = 5 ms: LOOP debe dar ~5.00 con 0 overruns. Si hay overruns, avisar.
- Velocidad real = VELOCITY (default dashboard 40 cm/s) en PI y SMC; la pendiente de la referencia debe dar ~VELOCITY.
- El Ki del PI se reescala por LOOP_MS/2.34 (periodo con que se sintonizo), asi el PI se comporta como antes.
Si el SMC oscila (comando de direccion agitado): subir phi (0.08-0.10). Si queda sesgado a un lado en arcos: calibrar psi.

## Calibrar psi (angulo de rueda efectivo a 100 %)
En un arco constante (tramo del medio del PATH de prueba), con SMC: si Sw tiene el mismo signo de forma sostenida,
psi_nuevo = psi * Eq / (Eq + Sw)  (usar los valores de las barras en %, con signo). Cargarlo en el campo psi y
Enviar SMC. Valores esperables: entre 17.5 (L/sin con r = 80) y 20.

## Rollback
Selector en PI y START: es exactamente el lazo anterior (el codigo del PI no cambio). Si hay que volver todo atras:
git checkout -- main_rs.py dashboard.html (lo corres vos) y borrar smc_control.py.

## Cambios en main_rs.py respecto del anterior (solo lectura con git diff)
- La odometria (x_local, y_local) se calcula ANTES del control en vez de despues (necesaria para e_y).
- Se integra la pose de la referencia (x_ref, y_ref) en el mismo bloque Euler de N ticks.
- Se calculan e_y y e_s siempre (PI y SMC) para reportar el error lateral; costo: unas cuatro funciones
  trigonometricas por tick (ver LOOP para el periodo real).
- Telemetria con 2 campos mas; el dashboard acepta tambien telemetria vieja de 15 campos.

## Subir al hub: archivo unico (main_bundle.py)
Para no subir 6 archivos sueltos: `python SMC/make_bundle.py` genera main_bundle.py en la raiz, con main_rs.py y todos
sus modulos adentro (sin imports locales). Se sube SOLO ese archivo (BlocklyPy Commander / Pybricks Code / pybricksdev).
Volver a correr el script despues de editar cualquier .py del hub. SMC/test_firmware_stub.py prueba los dos
(FW_MODULE=main_bundle para el bundle). Una sola conexion BLE a la vez: desconectar el Commander antes de conectar
el dashboard (y al reves).

## Perfil de velocidad de la referencia (dashboard v1.4)

Selector "Perfil de velocidad": Trapezoidal (default) o Constante. Comandos del hub: PROF CONST|TRAP y ACC a [cm/s^2] (default 80).
- Constante: la referencia arranca y frena con un escalon; la traccion (tau ~0.15 s) no lo sigue y el auto cruza el final a ~VELOCITY (en simulacion a 40 cm/s: ~40 cm/s, unos 6 cm de avance por inercia).
- Trapezoidal: sube con ACC hasta VELOCITY y frena con la misma rampa, solo al inicio del recorrido, al final y en los cambios de marcha; entre tramos del mismo sentido va a VELOCITY. En simulacion (VEL 40): a=50 cruza el final a 6 cm/s (1 cm de inercia), a=100 a 9 cm/s, a=200 a 28 cm/s. Cuesta tiempo: T 6.8 s (a=50), 6.1 s (a=100), 5.5 s (constante).
- Con SMC el perfil trapezoidal tambien baja el error: d_rms 0.33 -> 0.24 cm y d_max 1.84 -> 0.66 cm (a=50). El PI casi no cambia (d_rms 1.07 -> 1.16).
- A comparar en el auto: error de posicion al detenerse (cuanto avanza de mas despues del ultimo tramo) con Constante vs Trapezoidal a=80, mismo PATH y VEL 40.
