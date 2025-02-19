function new_path = timeflip(path)
    % timeflip transform described around the end of the article
    
    new_path = [];  % Inicializar el nuevo array de structs

    for i = 1:length(path)
        % Invertir el sentido del gear
        element = reverse_gear(path(i));  % Llamar a la función reverseGear
        new_path = [new_path, element];   % Añadir el elemento modificado
    end
end
