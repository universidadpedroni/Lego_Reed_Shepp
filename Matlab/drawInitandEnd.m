function drawInitandEnd(start_point, end_point, index)
    %% Función para dibujar el punto de inicio y fin
    % Dibuja la trayectoria Dubins en el espacio y simula el movimiento del vehículo.
    start_point(3) = deg2rad(start_point(3));
    end_point(3) = deg2rad(end_point(3));
    % Dibujar los puntos inicial y final
    plot(start_point(1), start_point(2), 'bo', 'LineWidth', 2);
    plot(end_point(1), end_point(2), 'bo', 'LineWidth', 2);

    %% Dibujo de los vectores de salida y llegada
    % Vector de salida
    quiver(start_point(1), start_point(2), cos(start_point(3)), sin(start_point(3)), 20, 'b', 'LineWidth', 2, 'MaxHeadSize', 2);
    text(start_point(1) + 1, start_point(2) + 1, strcat("Postura inicial ", num2str(index)));
    % Vector de llegada
    quiver(end_point(1), end_point(2), cos(end_point(3)), sin(end_point(3)), 20, 'b', 'LineWidth', 2, 'MaxHeadSize', 2);
    text(end_point(1) + 1, end_point(2) + 1, strcat("Postura final ", num2str(index)));
end

