L_car = 24; %[cm] Distancia entre ejes
psi_max = deg2rad(20); %[rad] Máximo ángulo de giro de las ruedas delanteras.
v = 39; %[cm/seg] Velocidad lineal constante del vehículo

%r_turn_min = L_car / sin(psi_max); %[cm] Mínimo radio de giro.
r_turn_min = 80.0;   % Forzamos el radio de giro un poco más grande para darle capacidad al control de dirección
