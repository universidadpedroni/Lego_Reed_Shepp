%% REEDS SHEPP CAR
close all; clear; clc
fprintf("Fuente: https://github.com/nathanlct/reeds-shepp-curves/blob/master/reeds_shepp.py")

%% Path
PATH = [0 0 0;...
        0 0 90;...
        %0 100 90;...
        %-50 50 45;...
        %0 0 0;
        ];

%% Constantes
run car_constants.m
%% Creación de la figura para los gráficos
%figure('units','normalized','outerposition',[0 0 1 1]); 
figure;
    hold on; grid on;
    xlabel('x [cm]'); ylabel('y [cm]');
    title(['Trayectorias de Reeds Shepp']);
    %extra = 20;
    %axis([min(PATH(:,1))-extra max(PATH(:,1))+extra min(PATH(:,2))-extra max(PATH(:,2))+extra])
%% Procesamiento
% Inicialización de variables de simulación
x_next_start = 0;
y_next_start = 0;
theta_next_start = 0;

for i = 1:size(PATH, 1) -1
    start_point = zeros(1,3);
    end_point = zeros(1,3);

    start_point_original = PATH(i,:);
    end_point_original = PATH(i+1,:);
    % Normalización de los puntos de inicio y fin
    start_point(1) = start_point_original(1) / r_turn_min;
    start_point(2) = start_point_original(2) / r_turn_min;
    start_point(3) = deg2rad(start_point_original(3));

    end_point(1) = end_point_original(1) / r_turn_min;
    end_point(2) = end_point_original(2) / r_turn_min;
    end_point(3) = deg2rad(end_point_original(3));

    

    % Búsqueda de las trayectorias
    paths = get_all_paths(start_point, end_point);
    min_distance_index = get_optimal_path(paths);
    path = paths{min_distance_index};
    
    % Desnormalización de las distancias
    for j = 1:length(path)
        path(j).distance = path(j).distance * r_turn_min; % Escalar la distancia real
    end
    
    % Simulación
    % Obtener las coordenadas de 'end' en el sistema de coordenadas donde 'start' es (0,0,0)
    
    drawInitandEnd(start_point_original, end_point_original, i)
    [x_next_start, y_next_start, theta_next_start] = drawReedsSheppTrajectory([x_next_start, y_next_start, theta_next_start], r_turn_min, path, 0.01, v);

end

disp('Terminado')