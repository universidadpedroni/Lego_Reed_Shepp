function path = path12(x, y, phi)
    % Formula 8.11: C|C[pi/2]SC[pi/2]|C
    
    % Incluir el archivo de constantes
    run('constants.m');
    
 
    path = [];  % Inicializar la variable path

    xi = x + sin(phi);
    eta = y - 1 - cos(phi);
    [rho, theta] = R(xi, eta);

    if rho >= 4
        u = sqrt(rho * rho - 4) - 4;
        A = atan2(2, u + 4);
        t = M(theta + pi/2 + A);
        v = M(t - phi);

        % Crear los elementos del path y añadirlos a la lista
        path = [path, createPathElement(t, LEFT, FORWARD)];
        path = [path, createPathElement(pi/2, RIGHT, BACKWARD)];
        path = [path, createPathElement(u, STRAIGHT, BACKWARD)];
        path = [path, createPathElement(pi/2, LEFT, BACKWARD)];
        path = [path, createPathElement(v, RIGHT, FORWARD)];
    end
end
