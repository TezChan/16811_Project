clear all;
close all;
clc;

% Create the map.
map = generateMap('map1.png', [20 20]);

% Create the robot parameters.
robot.radius = 1;
robot.linkage = 3;

%% Determine the start and goal nodes.
imshow('map1.png');
disp('Pick the location of the robots starting configuration');
[x, y] = ginput(2);
start = [round(x(1)/map.space(2)) round(y(1)/map.space(1)) atan2(x(2) - x(1), y(2) - y(1))];
disp('Pick the location of the robots final configuration');
[x, y] = ginput(2);
goal = [round(x(1)/map.space(2)) round(y(1)/map.space(1)) atan2(x(2) - x(1), y(2) - y(1))];
close all;

%% Generate the C-space graph.
[nodes, graph] = generateConfigGraph(robot, map, start, goal);

%% Path plan on the graph.
path = aStar(start, goal, nodes, graph);

%% Generate a video from the path planning.
% updateMap(robot, map, path);
for i=1:size(path, 1)
    close all;
    imagesc(map.obstacles);
    hold on;
    plot(path(i, 1), path(i, 2), 'k.');
    plot(path(i, 1) + 3*sin(path(i, 3)), path(i, 2) + 3*sin(path(i, 3)), 'kx');
    pause(0.5);
end