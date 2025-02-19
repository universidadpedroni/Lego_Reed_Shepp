function new_path = reflect(path)
    % reflect transform described around the end of the article
    
    new_path = [];  % Inicializar el nuevo array de structs

    for i = 1:length(path)
        % Invertir la dirección del steering
        element = reverse_steering(path(i));  % Llamar a la función reverseSteering
        new_path = [new_path, element];   % Añadir el elemento modificado
    end
end
