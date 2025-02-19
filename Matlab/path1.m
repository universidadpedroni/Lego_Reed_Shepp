function path = path1(x, y, phi)
    % Formula 8.1: CSC (same turns)
    run constants.m
    
    
    path = [];  % Usar celda en lugar de array

    [u, t] = R(x - sin(phi), y - 1 + cos(phi));
    v = M(phi - t);

    path = [path, createPathElement(t, LEFT, FORWARD)];
    path = [path, createPathElement(u, STRAIGHT, FORWARD)];
    path = [path, createPathElement(v, LEFT, FORWARD)];

end