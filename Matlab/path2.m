function path = path2(x, y, phi)
    % Formula 8.2: CSC (opposite turns)
    run constants.m
    phi = M(phi);
    path = []; % Inicializar como cell array vacío

    [rho, t1] = R(x + sin(phi), y - 1 - cos(phi));

    if rho * rho >= 4
        u = sqrt(rho * rho - 4);
        t = M(t1 + atan2(2, u));
        v = M(t - phi);

        path = [path, createPathElement(t, LEFT, FORWARD)];       % Usar array en lugar de celdas
        path = [path, createPathElement(u, STRAIGHT, FORWARD)];   % Agregar el nuevo elemento al array
        path = [path, createPathElement(v, RIGHT, FORWARD)];      % Agregar otro nuevo elemento
    end
end

