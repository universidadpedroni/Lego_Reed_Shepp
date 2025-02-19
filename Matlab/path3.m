function path = path3(x, y, phi)
    % Formula 8.3: C|C|C
    run constants.m
    
    path = []; % Inicializar como cell array vacío

    xi = x - sin(phi);
    eta = y - 1 + cos(phi);
    [rho, theta] = R(xi, eta);

    if rho <= 4
        A = acos(rho / 4);
        t = M(theta + pi/2 + A);
        u = M(pi - 2*A);
        v = M(phi - t - u);

       path = [path, createPathElement(t, LEFT, FORWARD)];      % Usar array en lugar de celdas
        path = [path, createPathElement(u, RIGHT, BACKWARD)];    % Agregar el nuevo elemento al array
        path = [path, createPathElement(v, LEFT, FORWARD)];      % Agregar otro nuevo elemento
    end
end

