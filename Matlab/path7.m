function path = path7(x, y, phi)
    % Formula 8.8: C|CuCu|C
    
    % Incluir el archivo de constantes
    run constants.m;

   
    path = [];  % Inicializar la variable path

    xi = x + sin(phi);
    eta = y - 1 - cos(phi);
    [rho, theta] = R(xi, eta);
    u1 = (20 - rho*rho) / 16;

    if rho <= 6 && 0 <= u1 && u1 <= 1
        u = acos(u1);
        A = asin(2 * sin(u) / rho);
        t = M(theta + pi/2 + A);
        v = M(t - phi);

        % Crear los elementos del path y añadirlos a la lista
        path = [path, createPathElement(t, LEFT, FORWARD)];
        path = [path, createPathElement(u, RIGHT, BACKWARD)];
        path = [path, createPathElement(u, LEFT, BACKWARD)];
        path = [path, createPathElement(v, RIGHT, FORWARD)];    

    end
end
