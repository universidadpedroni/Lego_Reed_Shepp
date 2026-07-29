function [x_end, y_end, theta_end] = simulateReedsSheppTrajectory(start_point, r_turn_min, path, h, v)
%% Función generalizada para todas las trayectorias CSC y CCC
% Dibuja la trayectoria Reeds-Shepp en el espacio y simula el movimiento del vehículo.
decimation_constant = 80;   % 1 auto/flecha cada este nº de pasos (subir = menos autos)
L_head = 5;                 % [cm] largo fijo de la flecha de heading (scale=0 abajo)
%% Dibujo de la trayectoria
x = start_point(1);
y = start_point(2);
theta = start_point(3);

time_pause = 0.001;
delta_s = h * v; % Distancia máxima recorrida por paso (paso de arco nominal)
hold on; % Mantén el gráfico para dibujar iterativamente
run constants.m
run car_constants.m

for i = 1:length(path)
    decimation_index = 0;

    % Nº ENTERO de pasos para cubrir el tramo, y paso de arco recalculado para
    % caer EXACTO al final del tramo (mismo criterio que wT=(qf-qi)/(N*h) de los
    % generadores articulares). Así cada elemento avanza su largo justo y gira su
    % ángulo justo (N*ds = distance), sin residuo que se propague al tramo siguiente.
    N  = ceil(path(i).distance / delta_s);
    ds = path(i).distance / N;              % ds <= delta_s

    for k = 1:N
        theta = theta - path(i).gear * path(i).steering * ds / r_turn_min; % Cambio angular por paso
        % El signo - indica que hay algun problema con la definición de LEFT y RIGHT
        x_new = x + path(i).gear * ds * cos(theta);
        y_new = y + path(i).gear * ds * sin(theta);

        plot([x, x_new], [y, y_new], 'g', 'LineWidth', 1);

        if mod(decimation_index, decimation_constant) == 0
            quiver(x, y, L_head*cos(theta), L_head*sin(theta), 0, 'r', 'LineWidth', 1.5, 'MaxHeadSize', 0.5);
            drawCar(x, y, theta, L_car, -psi_max);
        end

        x = x_new;
        y = y_new;
        decimation_index = decimation_index + 1;
        pause(time_pause);
    end

    % Dibujar el último auto del tramo (cae exacto en el fin del elemento)
    quiver(x, y, L_head*cos(theta), L_head*sin(theta), 0, 'r', 'LineWidth', 1.5, 'MaxHeadSize', 0.5);
    drawCar(x, y, theta, L_car, -psi_max);
    fprintf("Tramo L(%d): %.2f  ->  N=%d pasos de ds=%.3f\n", i, path(i).distance, N, ds);

end

x_end = x;
y_end = y;
theta_end = theta;

end
