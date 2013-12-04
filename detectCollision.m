function isCollision = detectCollision(robot, map, x, y, theta)
    % Create the template for a circular robot.
    [xs, ys] = meshgrid(-robot.radius:robot.radius, -robot.radius:robot.radius);
    z = xs.^2 + ys.^2;
    xs = xs(z <= robot.radius^2);
    ys = ys(z <= robot.radius^2);

    % Initialize a matrix to store the collision results.
    isCollision = false(size(x));

    % Iterate through each x-y-theta argument and check whether it is in
    % collision.
    for i=1:numel(x)
        % Check if the first robot is in collision with any objects.
        % Put the robot template at the position of the first robot.
        x1 = xs + x(i);
        y1 = ys + y(i);
        % Make sure the entire robot is within the map range.
        if sum(x1 < 1 | y1 < 1 | x1 > map.size(2) | y1 > map.size(1))
            isCollision(i) = true;
            continue;
        end
        % Check for collisions.
        if trace(map.obstacles(y1, x1)) > 0
            isCollision(i) = true;
            continue;
        end

        % Check if the second robot is in collision with the object.
        % Put the robot template at the position of the second robot.
        x2 = round(xs + x(i) + robot.linkage*sin(theta(i)));
        y2 = round(ys + y(i) + robot.linkage*cos(theta(i)));
        % Make sure the entire robot is within the map range.
        if sum(x2 < 1 | y2 < 1 | x2 > map.size(2) | y2 > map.size(1))
            isCollision(i) = true;
            continue;
        end
        % Check for collisions.
        if trace(map.obstacles(y2, x2)) > 0
            isCollision(i) = true;
            continue;
        end

        % Check if the linkage is in collision with the object.
    end
end