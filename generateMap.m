function map = generateMap(map_image, world_size)
% Takes an image and computes a binary map of the obstacles.
% 
% INPUTS:
% map_image: filename of the PNG image containing the map
% world_size: [x, y] physical size of the world represented by the map.
% (pixels per ft represented by the map). 
% 
% OUTPUTS:
% map: binary map representing where the objects are.

    I = imread(map_image);
    I = rgb2gray(I);
    
    imsize = size(I);

    x = floor(linspace(1, size(I, 1), world_size(2)));
    y = floor(linspace(1, size(I, 2), world_size(1)));

    I = I(x', y');
    map.obstacles = I < 20;
    map.size = world_size;
    map.space = imsize ./ [world_size(2) world_size(1)];
end