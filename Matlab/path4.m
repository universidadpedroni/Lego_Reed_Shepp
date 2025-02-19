function path = path4(x, y, phi)
    % Formula 8.4 (1): C|CC
    run constants.m
    
    path = []; % Inicializar como cell array vacío

    xi = x - sin(phi);
    eta = y - 1 + cos(phi);
    [rho, theta] = R(xi, eta);

    if rho <= 4
        A = acos(rho / 4);
        t = M(theta + pi/2 + A);
        u = M(pi - 2*A);
        v = M(t + u - phi);

        path = [path, createPathElement(t, LEFT, FORWARD)];      % Usar array en lugar de celdas
        path = [path, createPathElement(u, RIGHT, BACKWARD)];    % Agregar el nuevo elemento al array
        path = [path, createPathElement(v, LEFT, BACKWARD)];     % Agregar otro nuevo elemento
    end
end
