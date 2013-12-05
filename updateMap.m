function updateMap(robot, map, path)
% Iterates through a file and repeatedly draws the map and the robot's
% positions. Saves the animation to a movie file.

if nargin < 3
    % Open the text file
    fid = fopen('cspace_points.txt');
    
    % Read the state of the robots from the file.
    [state, num_states] = fscanf(fid, '%f %f %f');

    % Close the open file.
    fclose(fid);
    
    % Reshape the state matrix to be a nx3 (x y theta).
    state = reshape(state, 3, num_states/3);
else
    % The states of the robot is just its path.
    state = path;
end
    
    % Create a movie object to save movie to.
    mov = VideoWriter('results.avi');
    mov.FrameRate = 2;
    open(mov);
    
    % Iterate through the state array and create an image for each position
    % of the robot. Save those to a movie file.
    for i=1:size(state, 1);
        % Draw the map.
        imshow('map1.png');
        hold on;
        
        % Draw the robots.
        drawRobots(robot, map, state(i,1), state(i,2), state(i,3));
        
        % Get the current plot as a movie frame and add it to the video.
        frame = getframe;
        writeVideo(mov, frame);
        
        % Close the active plot.
        close all;
    end

    % Close the movie file.
    close(mov);
end