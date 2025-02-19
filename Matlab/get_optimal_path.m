function min_distance_index = get_optimal_path(paths)
    min_distance = inf;
    min_distance_index = -1;
    
    for i = 1:length(paths)
        % Suponiendo que `paths{i}.distance` puede tener 3, 4 o 5 elementos
        path_distance = 0;
        for j = 1: length(paths{i})
            
            path_distance = path_distance + paths{i}(j).distance;
        end
        
        if path_distance < min_distance
            min_distance = path_distance;
            min_distance_index = i;
        end
    end
    
end

