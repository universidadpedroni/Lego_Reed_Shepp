function path = path10(x, y, phi)
    % Formula 8.10 (1): C|C[pi/2]SC
    
    % Incluir el archivo de constantes
    run('constants.m');
    
    
    path = [];  % Inicializar la variable path

    xi = x + sin(phi);
    eta = y - 1 - cos(phi);
    [rho, theta] = R(xi, eta);

    if rho >= 2
        t = M(theta + pi/2);
        u = rho - 2;
        v = M(phi - t - pi/2);

        % Crear los elementos del path y añadirlos a la lista
        path = [path, createPathElement(t, LEFT, FORWARD)];       % Añadir el primer elemento al array
        path = [path, createPathElement(pi/2, RIGHT, BACKWARD)];   % Añadir el segundo elemento al array
        path = [path, createPathElement(u, STRAIGHT, BACKWARD)];   % Añadir el tercer elemento al array
        path = [path, createPathElement(v, RIGHT, BACKWARD)];      % Añadir el cuarto elemento al array

    end
end
