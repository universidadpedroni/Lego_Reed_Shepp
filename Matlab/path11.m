function path = path11(x, y, phi)
    % Formula 8.10 (2): CSC[pi/2]|C
    
    % Incluir el archivo de constantes
    run('constants.m');
    
    
    path = [];  % Inicializar la variable path

    xi = x + sin(phi);
    eta = y - 1 - cos(phi);
    [rho, theta] = R(xi, eta);

    if rho >= 2
        t = M(theta);
        u = rho - 2;
        v = M(phi - t - pi/2);

        % Crear los elementos del path y añadirlos a la lista
        path = [path, createPathElement(t, LEFT, FORWARD)];         % Añadir el primer elemento al array
        path = [path, createPathElement(u, STRAIGHT, FORWARD)];     % Añadir el segundo elemento al array
        path = [path, createPathElement(pi/2, LEFT, FORWARD)];      % Añadir el tercer elemento al array
        path = [path, createPathElement(v, RIGHT, BACKWARD)];       % Añadir el cuarto elemento al array

    end
end
