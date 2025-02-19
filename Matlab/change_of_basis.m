function [new_x, new_y, new_theta] = change_of_basis(p1, p2)
    % Given p1 = (x1, y1, theta1) and p2 = (x2, y2, theta2) represented in a
    % coordinate system with origin (0, 0) and rotation 0 (in radians), return
    % the position and rotation of p2 in the coordinate system whose origin
    % is (x1, y1) and rotation is theta1.
    
    %theta_original = p1(3);  % Keep in radians
    dx = p2(1) - p1(1);  % Calculate the difference in x
    dy = p2(2) - p1(2);  % Calculate the difference in y
    
    % Corrected transformation equations
    new_x = dx * cos(p1(3)) + dy * sin(p1(3));
    new_y = - dx * sin(p1(3)) + dy * cos(p1(3));
    %new_theta = p2(3) - p1(3);  % Angle difference
    new_theta = M(p2(3) - p1(3));  % Angle difference, normalizado en -pi, pi
    
end


