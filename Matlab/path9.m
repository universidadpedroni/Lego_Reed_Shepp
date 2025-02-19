function path = path9(x, y, phi)
    % Formula 8.9 (2): CSC[pi/2]|C
    
    % Incluir el archivo de constantes
    run constants.m ;

   
    path = [];  % Inicializar la variable path

    xi = x - sin(phi);
    eta = y - 1 + cos(phi);
    [rho, theta] = R(xi, eta);

    if rho >= 2
        u = sqrt(rho*rho - 4) - 2;
        A = atan2(u+2, 2);
        t = M(theta + pi/2 - A);
        v = M(t - phi - pi/2);

        % Crear los elementos del path y añadirlos a la lista
       path = [path, createPathElement(t, LEFT, FORWARD)];        % Añadir el primer elemento al array
       path = [path, createPathElement(u, STRAIGHT, FORWARD)];    % Añadir el segundo elemento al array
       path = [path, createPathElement(pi/2, RIGHT, FORWARD)];    % Añadir el tercer elemento al array
       path = [path, createPathElement(v, LEFT, BACKWARD)];       % Añadir el cuarto elemento al array

    end
end
