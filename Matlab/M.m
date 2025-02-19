function theta = M(theta)
    % Devuelve el ángulo phi = theta mod (2*pi) en el rango -pi <= phi < pi.
    theta = mod(theta, 2*pi);
    if theta < -pi
        theta = theta + 2*pi;
    elseif theta >= pi
        theta = theta - 2*pi;
    end
        
end
