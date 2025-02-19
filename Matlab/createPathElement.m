function element = createPathElement(distance, steering, gear)
    element.distance = abs(distance);
    element.steering = steering;
    element.gear = gear;

    if distance < 0
        element = reverse_gear(element);
    end
end

