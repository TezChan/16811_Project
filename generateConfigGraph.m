function [nodes, graph] = generateConfigGraph(robot, map)
    % Sample the C-space thouroughly,
    [xs, ys, thetas] = meshgrid(1:map.size(1), 1:map.size(2), 0:.1:2*pi+.1);
    [l, w, h] = size(xs);
    
    % Create a structure to hold the nodes.
    nodes = [xs(:) ys(:) thetas(:)];
        
    % Find the points in C-space that are not in collision.
    collision = detectCollision(robot, map, xs(:), ys(:), thetas(:));    

    % Determine the edges.
    % Create a graph with no edges.
    graph = sparse(l*w*h, l*w*h);
    % Iterate through the sampled Cspace, and check for collisions and add
    % edges accordingly. Edges are only formed between 26pt connectivity of
    % 3D space.
    for k=1:h
        for i=1:l
            for j=1:w
                % Get a cuboid for the neighbors around point (i, j, k).
                [neighbor_x, neighbor_y, neighbor_z] = ...
                        meshgrid(max(1, i-1):min(i+1, l),...
                                 max(1, j-1):min(j+1, w),...
                                 max(1, k-1):min(k+1, h));
                % Compute the index of each neighbor in the nodes list.
                neighbors = neighbor_z(:)*w*l + neighbor_x*w + neighbor_y + 1;
                % Find the good neighbors, that are not in collision.
                good_neighbors = not(collision(neighbors));
                % Calculate the distances between the current node and its
                % neighbors.
                distances = pdist2(nodes(i*j*k), nodes(neighbors), @node_dist);
                % Add the distances from the current node to all its
                % neighbors into the graph.
                graph(i*j*k, neighbors) = good_neighbors.*distances;
                graph(neighbors, i*j*k) = (good_neighbors.*distances)';
            end
        end
    end


    function distance = node_dist(nodei, nodej)
        % Distance for each node is the sum of the distances each robot
        % travels.
        distance = norm(nodei(1:2) - nodej(1:2)) + norm(nodei(3:4) - nodej(3:4));
    end
end