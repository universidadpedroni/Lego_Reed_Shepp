function path = path8(x, y, phi)
    % Formula 8.9 (1): C|C[pi/2]SC
    
    % Incluir el archivo de constantes
    run constants.m;

    
    path = [];  % Inicializar la variable path

    xi = x - sin(phi);
    eta = y - 1 + cos(phi);
    [rho, theta] = R(xi, eta);

    if rho >= 2
        u = sqrt(rho*rho - 4) - 2;
        A = atan2(2, u+2);
        t = M(theta + pi/2 + A);
        v = M(t - phi + pi/2);

        % Crear los elementos del path y añadirlos a la lista
        path = [path, createPathElement(t, LEFT, FORWARD)];
        path = [path, createPathElement(pi/2, RIGHT, BACKWARD)];
        path = [path, createPathElement(u, STRAIGHT, BACKWARD)];
        path = [path, createPathElement(v, LEFT, BACKWARD)];

    end
end
