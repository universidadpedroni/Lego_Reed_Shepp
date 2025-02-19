function drawCar(x, y, theta, L_car, phi)
%% Fuente: https://jckantor.github.io/CBE30338/07.06-Path-Planning-for-a-Simple-Car.html    
scl = 0.4;      % Escala
%phi = 0; 
w = 8;  % Distancia de la rueda al centro del vehículo
r = 4;  % Radio de la rueda
    % Matriz de rotación
    R = [cos(theta), -sin(theta); sin(theta), cos(theta)];

    % Coordenadas del auto (sin rotar ni escalar)
    car = [
        r, w;
       -r, w;
        0,   w;
        0,  -w;
        r, -w;
       -r, -w;
        0,  -w;
        0,   0;
        L_car,   0;
        L_car,   w;
        L_car + r*cos(phi),  w + r*sin(phi);
        L_car - r*cos(phi),  w - r*sin(phi);
        L_car,   w;
        L_car,  -w;
        L_car + r*cos(phi), -w + r*sin(phi);
        L_car - r*cos(phi), -w - r*sin(phi)
    ];

    % Aplicar rotación y escala
    car_transformed = scl * (R * car')';

    % Graficar el auto
    plot(x + car_transformed(:,1), y + car_transformed(:,2), 'k', 'LineWidth', 2);
    hold on;
    plot(x, y, 'k.', 'MarkerSize', 10); % Dibuja el centro del auto
    axis equal;
end
