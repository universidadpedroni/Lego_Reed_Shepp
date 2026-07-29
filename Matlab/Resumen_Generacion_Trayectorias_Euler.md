# Generación de trayectorias por Euler — resumen y criterio

Resumen de lo trabajado sobre los generadores de trayectoria (robots articulares y auto Ackermann/Reeds-Shepp), pensado para reaplicarlo en el proyecto del robot Lego.

## La idea central

El movimiento se integra por Euler explícito con velocidad constante por tramo (perfil rectangular): el estado nuevo es el viejo más la velocidad por el paso. El punto fino, que es lo que resuelve casi todos los problemas, es cómo se termina cada tramo.

El paso `h` es FIJO: es el período de muestreo. En un micro es el intervalo de la interrupción del timer que ejecuta la tarea, no se toca. Lo que se ajusta es la velocidad.

Por eso, en lugar de cortar con un `while` por umbral (que se pasa del destino, deja residuo y puede colgar), se cuantiza el tiempo a un número entero de pasos y se recalcula la velocidad para caer exacto:

    T  = tiempo mínimo del tramo (respetando la vel. máxima)
    N  = ceil(T / h)                 % nº entero de muestras del timer
    wT = (qf - qi) / (N*h)           % velocidad recalculada -> cierra EXACTO

Con eso, en `N` pasos de `h` el estado cae clavado en `qf`, la velocidad queda constante todo el tramo, y como `N*h >= T` nunca se supera la velocidad máxima (el robot tarda a lo sumo un `h` más que el `T` teórico, nunca acelera de más).

La regla práctica: `for i = 1:N`, nunca `while (abs(error) >= delta)`.

## Los tres modos articulares

Independiente (secuencial, camino en "L"): un eje a la vez, `N1` y `N2` por eje, se encadenan por índice, total `N1+N2+1` filas.

Simultáneo a velocidad máxima: todos arrancan juntos, cada eje a su `wmax`, cada uno frena en su propio `Nk`; total `max(Nk)+1` filas.

Coordinado sincronizado: todos arrancan y llegan juntos, `T = max(|qf-qi|./wmax)` común, `wT = (qf-qi)/(N*h)` ecualizada; cierra exacto en la vía sin clamp.

## Auto Ackermann / Reeds-Shepp

Mismo criterio pero con paso de arco `delta_s = h*v` en lugar de `h`. Por cada elemento del path:

    N  = ceil(distance / delta_s)
    ds = distance / N               % paso de arco que cierra el tramo justo
    for k = 1:N: integrar con ds

El modelo del auto usa Euler semi-implícito (Euler-Cromer): primero actualiza el heading y mueve `x,y` con el heading NUEVO. Es una variante válida y algo más precisa para el vehículo; no es idéntica al forward puro de los robots articulares, pero el criterio de cierre exacto es el mismo. Con el `for` cada tramo gira exactamente `distance/r` y no arrastra deriva al tramo siguiente (antes, con el `while`, el error de heading se propagaba al sembrar el próximo tramo).

## Traducción a firmware embebido (robot Lego real)

Separar planificador e ISR:

Planificador (tarea de fondo, al recibir la orden de moverse): calcular una sola vez `T`, `N = ceil(T/h)` y el incremento por tick `dq = wT*h` (o `ds`). Todo lo caro —los `max`, la división— va acá, no en la interrupción.

ISR del timer (período `h`): `q += dq`, un solo add por eje. Sin división, sin `ceil`, sin ramas. Tiempo de ejecución constante y acotado, que es lo que se quiere dentro de una interrupción.

Dos cuidados de implementación real:
- No acumular `q += dq` en float indefinidamente: sobre muchos ticks deriva por redondeo. Llevar un contador de ticks `k` y al llegar a `k == N` forzar `q = qf` para matar el error acumulado (un `if` que corre una sola vez por tramo).
- Sin FPU, dejar `dq` en punto fijo (Q15/Q31) precalculado y la ISR en enteros. En STM32 con FPU, float simple y listo.

## Detalles de la visualización

Vector de tiempo para graficar q(t) y dq(t): tiene que haber UN tiempo por muestra, con largo igual a la cantidad de filas de la trayectoria. El total escalar `T_total` NO sirve. Además, si se concatenan tramos reincluyendo la fila inicial de cada uno, hay muestras repetidas, así que tampoco cierra `0:h:T_total`. Lo robusto:

    tvec = 0 : h : (size(q_total,1)-1)*h;

Flechas (quiver): no confiar en el autoescalado. Meter el largo dentro del vector y usar `scale = 0`, así mide exacto lo pedido, y bajar `MaxHeadSize`:

    quiver(x, y, L*cos(th), L*sin(th), 0, 'b', 'LineWidth', 1.5, 'MaxHeadSize', 0.5);

Densidad de dibujo: controlarla con un `decimation_constant` (dibujar 1 de cada N pasos) para que no se encimen los autos/marcadores.

Ojo con `pause()` sin argumento: no espera un tiempo, espera una tecla. Bloquea la ejecución con la figura vacía hasta que se apriete algo.

## Criterio de validación

Articular: `norm(q(end,:) - qf) ~ 0` por tramo; `|wT(k)| <= wmax(k)`; filas `= N+1` (o `N1+N2+1`).

Reeds-Shepp: heading final de cada elemento `= distance/r` exacto; el auto cierra sobre los waypoints del `PATH`.

## Organización

Las funciones genéricas (generadores, dibujo, simetrías) conviene tenerlas en una única carpeta compartida (tipo `Func_Comunes`) que cada proyecto referencia, y que cada robot conserve solo su `car_constants`/`main`. Mantener copias por proyecto lleva a que se desincronicen (fue justo lo que pasó entre las carpetas Ackermann y Lego).
