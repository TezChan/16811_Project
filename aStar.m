%% A* Search Algorithm
function path = aStar(start, goal, nodes, graph)
    % Initialize A* Variables
    closed = [];
    open = [start];
    score = [0 + heuristic(start, goal)];
    parent = zeros(size(graph,1), 1);

    % Perform A*
    while ~isempty(open)
        % Find the next node to explore. This is the one with the
        % lowest score.
        [~, ind] = min(score);
        current = open(ind, :);
        myscore = score(ind) - heuristic(current, goal);

        % Make sure we are not done.
        if (current == goal)
            path = reconstructPath(parent, current, nodes);
            return;
        end

        % Explore the current node.
        open = open([1:ind-1 ind+1:end], :);
        score = score([1:ind-1 ind+1:end]);
        closed = [closed; current];

        % Update the score for each neighbor.
        ind = findNode(current, nodes);
        neighbors = graph(ind, :);
        for i=1:length(neighbors)
            % Skip if it is not actually a neighbor.
            if (neighbors(i) == 0)
                continue;
            end
            % Skip if it is already closed.
            if ~isempty(findNode(nodes(i, :), closed))
                continue;
            end
            % Check if the neighbor is in the openset.
            j = findNode(nodes(i, :), open);
            new_score = myscore+graph(ind,i)+heuristic(nodes(i, :),goal);
            if isempty(j)  % Not open
                % Add it to the open set and mark us as the parent.
                open = [open; nodes(i, :)];
                score = [score; new_score];
                parent(i) = ind;
            else  % Already open.
                % Check if the score is better, then update node
                % accordingly.
                if (new_score < score(j))
                    % The score is better so update the score and mark
                    % us as the parent.
                    score(j) = new_score;
                    parent(i) = ind;
                end
            end
        end            
    end

    path = [];

    % Finds a node in a given set
    function ind = findNode(node, set)
        n = size(set, 1);
        a = repmat(node, n, 1) - set;
        ind = find(a(:,1) == 0 & a(:,2) == 0 & a(:,3) == 0);
    end

    % Reconstruct the path.
    function path = reconstructPath(parent, last_node, nodes)
        % Get index of last node.
        index = findNode(last_node, nodes);
        path = [];
        % The start node has no parent. Keep iterating till we get back
        % to the start node.
        while index ~= 0
            % Add the current node to the beginning of the path.
            path = [nodes(index, :); path];
            % Make the parent the current node.
            index = parent(index);
        end            
    end

    % Heuristic from a node to the goal.
    function dist = heuristic(point, goal)
        robot2i = point(1:2) + 15*[sin(point(3)) cos(point(3))];
        robot2j = goal(1:2) + 15*[sin(goal(3)) cos(goal(3))];
        dist = norm(point(1:2) - goal(1:2)) + norm(robot2i - robot2j);
    end
end