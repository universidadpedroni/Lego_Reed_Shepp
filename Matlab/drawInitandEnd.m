function drawInitandEnd(start_point, end_point, index)
    %% Función para dibujar el punto de inicio y fin
    % Dibuja la trayectoria Dubins en el espacio y simula el movimiento del vehículo.
    start_point(3) = deg2rad(start_point(3));
    end_point(3) = deg2rad(end_point(3));
    % Dibujar los puntos inicial y final
    plot(start_point(1), start_point(2), 'bo', 'LineWidth', 2);
    plot(end_point(1), end_point(2), 'bo', 'LineWidth', 2);

    %% Dibujo de los vectores de salida y llegada
    % Largo fijo de la flecha [cm]. Metemos el largo dentro del vector y usamos
    % scale = 0 (autoescalado de quiver apagado) para que mida exacto esto.
    L_arrow = 10;   % probar 0.2*r_turn_min si se quiere que escale con el robot
    % Vector de salida
    quiver(start_point(1), start_point(2), L_arrow*cos(start_point(3)), L_arrow*sin(start_point(3)), ...
           0, 'b', 'LineWidth', 1.5, 'MaxHeadSize', 0.5);
    text(start_point(1) + 1, start_point(2) + 1, strcat("Postura inicial ", num2str(index)));
    % Vector de llegada
    quiver(end_point(1), end_point(2), L_arrow*cos(end_point(3)), L_arrow*sin(end_point(3)), ...
           0, 'b', 'LineWidth', 1.5, 'MaxHeadSize', 0.5);
    text(end_point(1) + 1, end_point(2) + 1, strcat("Postura final ", num2str(index)));
end
