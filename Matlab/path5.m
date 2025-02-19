function path = path5(x, y, phi)
    % Formula 8.4 (2): CC|C
    run constants.m
    
    
    path = [];  % Inicializar como cell array vacío

    xi = x - sin(phi);
    eta = y - 1 + cos(phi);
    [rho, theta] = R(xi, eta);

    if rho <= 4
        u = acos(1 - rho^2 / 8);
        A = asin(2 * sin(u) / rho);
        t = M(theta + pi/2 - A);
        v = M(t - u - phi);

        % Crear los elementos del path y añadirlos a la celda
        path = [path, createPathElement(t, LEFT, FORWARD)];
        path = [path, createPathElement(u, RIGHT, FORWARD)];
        path = [path, createPathElement(v, LEFT, BACKWARD)];

    end
end

