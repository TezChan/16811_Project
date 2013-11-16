function updateMap()
% Iterates through a file and repeatedly draws the map and the robot's
% positions. Saves the animation to a movie file.

    % Open the text file
    fid = fopen('robot_state.txt');
    
    % Read the state of the robots from the file.
    [state, num_states] = fscanf(fid, '%f %f %f');

    % Close the open file.
    fclose(fid);
    
    % Reshape the state matrix to be a nx3 (x y theta).
    state = reshape(state, 3, num_states/3)';
    
    % Draw the map for the first time and save the data vector so we can
    % repeatedly draw the map.
    data = drawMap('map1.txt');
    
    % Create a movie object to save movie to.
    mov = VideoWriter('results.avi');
    mov.FrameRate = 2;
    open(mov);
    
    % Iterate through the state array and create an image for each position
    % of the robot. Save those to a movie file.
    for i=1:size(state, 1);
        % Draw the robots.
        drawRobots(state(i,1), state(i,2), state(i,3));
        
        % Get the current plot as a movie frame and add it to the video.
        frame = getframe;
        writeVideo(mov, frame);
        
        % Close the active plot.
        close all;
        
        % Draw the map for the next time in.
        drawMap('', data);
    end

    % Close the movie file.
    close(mov);
end