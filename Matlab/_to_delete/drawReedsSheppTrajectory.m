function [x_end, y_end, theta_end] = drawReedsSheppTrajectory(start_point, r_turn_min, path, h, v)
%% Función generalizada para todas las trayectorias CSC y CCC
% Dibuja la trayectoria Dubins en el espacio y simula el movimiento del vehículo.
decimation_constant = 40;
vector_size = 1;
%% Dibujo de la trayectoria
x = start_point(1); 
y = start_point(2); 
theta = start_point(3);

time_pause = 0.001;
delta_s = h * v; % Distancia recorrida por paso
hold on; % Mantén el gráfico para dibujar iterativamente
run constants.m
run car_constants.m

for i = 1:length(path)
    decimation_index = 0;
    encoder = 0;

    while(abs(encoder) <= path(i).distance)
        theta = theta - path(i).gear * path(i).steering * delta_s / r_turn_min; % Cambio angular basado en distancia
        % El signo - indica que hay algun problema con la definición de
        % LEFT y RIGHT
        x_new = x + path(i).gear *  delta_s * cos(theta);
        y_new = y + path(i).gear *  delta_s * sin(theta);
        
        plot([x, x_new], [y, y_new], 'g', 'LineWidth', 1);
        % Simulación del avance del encoder del vehículo.
        encoder = encoder + delta_s;
        if mod(decimation_index, decimation_constant) == 0
            quiver(x, y, cos(theta), sin(theta), vector_size, 'r', 'LineWidth', 2, 'MaxHeadSize', 2);
            drawCar(x, y, theta, L_car, -psi_max);
            %fprintf("Encoder: %.2f \t L(%d): %.2f\n", encoder, i, path(i).distance);
            
        end

        x = x_new; 
        y = y_new;
        decimation_index = decimation_index + 1;
        pause(time_pause);
        
    end
    % Dibujar el último auto
quiver(x, y, cos(theta), sin(theta), vector_size, 'r', 'LineWidth', 2, 'MaxHeadSize', 2);
drawCar(x, y, theta, L_car, -psi_max);
% fprintf("Encoder: %.2f \t L(%d): %.2f\n", encoder, i, path(i).distance);

end

    
x_end = x;
y_end = y;
theta_end = theta;





end
