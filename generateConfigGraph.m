function [nodes, graph] = generateConfigGraph(robot, map, start, goal)
    % Sample the C-space thouroughly,
    [xs, ys, thetas] = meshgrid(1:map.size(1), 1:map.size(2), 0:.1:2*pi+.1);
    [l, w, h] = size(xs);
    
    % Create a structure to hold the nodes.
    nodes = [start; goal; xs(:) ys(:) thetas(:)];
    
    % Find the points in C-space that are not in collision.
    collision = detectCollision(robot, map, nodes(:,1), nodes(:,2), nodes(:,3));    

    % Determine the edges.
    % Insert edges for the start and goal nodes.
    nearest_neighbors = knnsearch(nodes(3:end, :), nodes(1:2, :), 'K', 1);
    x = [1; 2];
    y = nearest_neighbors+2;
    value = [1; 1];
    % Iterate through the sampled Cspace, and check for collisions and add
    % edges accordingly. Edges are only formed between 26pt connectivity of
    % 3D space.
    % Initialize timer for wait estimate.
    timer1 = tic();
    last_time = 0;
    for k=1:h
        % Compute the wait estimate so the user can know approximately how
        % long things will take.
        if(toc(timer1)-last_time>10)
            last_time = toc(timer1);
            time_proj = (h-k+1)/(k-1) *last_time;
            display(strcat('estimated completion in  ', num2str(time_proj),' seconds'));
        end
        
        for i=1:l
            for j=1:w                
                % Compute the index for the current node.
                index = (k-1)*w*l + (i-1)*w + j + 2;
                % Check if it is in collision, because then we can skip
                % this point.
                if collision(index)
                    continue;
                end
                % Get a cuboid for the neighbors around point (i, j, k).
                [neighbor_x, neighbor_y, neighbor_z] = ...
                        meshgrid(max(1, i-1):min(i+1, l),...
                                 max(1, j-1):min(j+1, w),...
                                 max(1, k-1):min(k+1, h));
                % Account for 1-indexing in matlab.
                neighbor_x = neighbor_x - 1;
                neighbor_y = neighbor_y - 1;
                neighbor_z = neighbor_z - 1;
                % Compute the index of each neighbor in the nodes list.
                neighbors = neighbor_z(:)*w*l + neighbor_x(:)*w + neighbor_y(:) + 3;
                % Find the good neighbors, that are not in collision.
                good_neighbors = not(collision(neighbors));
                % Calculate the distances between the current node and its
                % neighbors.
                distances = pdist2(nodes(index, :), nodes(neighbors, :), @node_dist);
                % Add the distances from the current node to all its
                % neighbors into the graph.
                if (sum(good_neighbors) > 0)
                    x = [x; repmat(index, size(neighbors,1), 1)];
                    y = [y; neighbors];
                    value = [value; good_neighbors.*distances'];
                end
            end
        end
    end
    toc(timer1)
    % Create the sparse graph.
    graph = sparse([x; y], [y; x], [value; value], l*w*h+2, l*w*h+2);

    function distance = node_dist(nodei, nodej)
        % Distance for each node is the sum of the distances each robot
        % travels.
        % Calculate the position of robot1.
        robot1i = nodei(1:2);
        robot1j = nodej(1:2);
        
        % Calculate the position of robot2.
        robot2i = nodei(1:2) + robot.linkage*[sin(nodei(3)) cos(nodei(3))];
        robot2j = nodej(1:2) + robot.linkage*[sin(nodej(3)) cos(nodej(3))];
        
        % Distance is the sum of the distances each robot travels.
        distance = norm(robot1i - robot1j) + norm(robot2i - robot2j);
    end
end