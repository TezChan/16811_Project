function data = drawMap(filename, data)
% Creates a new figure and draws the world map, by either reading from a
% data file, or parsing a pre-computed datadump.
% 
% INPUTS
% filename: name of the file that you would like to plot from
% data: pre-read data that you would like to plot
% OUTPUTS
% Returns the data that is read that can be used to plot the map
% repeatedly.
%
% USAGE
% data = drawMap('map1.txt');
% drawMap('', data);
    if (nargin < 2)  % Plot from the file.
        % Open the text file
        fid = fopen(filename);

        figure;
        hold on;
        axis([0 100 0 100]);
        
        data = [];

        % Read from the file until the next # symbol and draw each obstacle
        % until the end of file is reached.
        while (~feof(fid))
            % Skip the current # symbol.
            fscanf(fid, '# %s %d');
            % Read all the corners contained in the map file for this particular
            % object. Reads until the next # symbol.
            [corners, num_c] = fscanf(fid, '%f %f');
            
            % Reshape corners into a nx2 matrix.
            num_c = num_c/2;
            corners = reshape(corners, 2, num_c)';
            % Add the first element to the end to draw a closed object.
            corners = [corners; corners(1, :)];
            
            % Plot the object onto the map.
            plot(corners(:,1), corners(:,2), 'k');
            
            % Add the corners into the data object so we dont need to open
            % and read the text file every time.
            data = [data; corners];
        end

        % Close the text file.
        fclose(fid);
    else  % Plot directly from data
        % Create initial world map
        figure;
        hold on;
        axis([0 100 0 100]);
        
        % Counter to keep track of start of next object to plot.
        i = 1;
        
        % While there are still objects left to plot, keep plotting.
        while (i < length(data))
            % Find the first element of this object.
            first = data(i, :);
            
            % The last element is the repeat of the first one, that is past
            % the current value of i.
            last = find(data(:,1) == first(1) & data(:,2) == first(2));
            last = last(last > i);
            last = last(1);
            
            % Plot all the corners between the first and last element
            % inclusively.
            plot(data(i:last, 1), data(i:last, 2), 'k');
            
            % Update the counter to point past the plotted elements.
            i = last+1;
    end
end