%% Testing
close all; clear; clc
run car_constants.m


%% Creación de la figura para los gráficos
% figure('units','normalized','outerposition',[0 0 1 1]); 
%     hold on; grid on;
%     xlabel('x [cm]'); ylabel('y [cm]');
%     title(['Trayectorias de Reeds Shepp']);


end_point = zeros(1,3);

start_point = [0, 0, 0];
end_point_original = [120, 60, -90];

end_point(1) = end_point_original(1) / r_turn_min;
end_point(2) = end_point_original(2) / r_turn_min;
end_point(3) = deg2rad(end_point_original(3));

paths = get_all_paths(start_point, end_point);
min_distance_index = get_optimal_path(paths);
path = paths{min_distance_index};

for i = 1:length(path)
    path(i).distance = path(i).distance * r_turn_min; % Escalar la distancia real
end

%drawInitandEnd(start_point, end_point_original, 0)
%drawReedsSheppTrajectory(start_point, r_turn_min, path, 0.01, v)

disp('Terminado')

