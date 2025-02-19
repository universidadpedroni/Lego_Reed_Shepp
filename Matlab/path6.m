function path = path6(x, y, phi)
    % Formula 8.7: CCu|CuC
    run constants.m
    
    
    path = [];  % Inicializar la variable path

    xi = x + sin(phi);
    eta = y - 1 - cos(phi);
    [rho, theta] = R(xi, eta);

    if rho <= 4
        if rho <= 2
            A = acos((rho + 2) / 4);
            t = M(theta + pi/2 + A);
            u = M(A);
            v = M(phi - t + 2*u);
        else
            A = acos((rho - 2) / 4);
            t = M(theta + pi/2 - A);
            u = M(pi - A);
            v = M(phi - t + 2*u);
        end

        % Crear los elementos del path y añadirlos a la lista
        path = [path, createPathElement(t, LEFT, FORWARD)];
        path = [path, createPathElement(u, RIGHT, FORWARD)];
        path = [path, createPathElement(u, LEFT, BACKWARD)];
        path = [path, createPathElement(v, RIGHT, BACKWARD)];

    end
end
