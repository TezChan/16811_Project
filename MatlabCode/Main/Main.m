% Neil Abcouwer and Priya Deo
% 16-811 Project
% 11/14/2013
%
% Main file for A star

clear all
close all
clc

%get map
map = imread('map1.png');
map = im2bw(map,.5);
figure
imagesc(map)
colormap gray

g_score = NaN(size(map));
f_score = NaN(size(map));
openset = zeros(size(map));
closedset = zeros(size(map));
camefrom = zeros(size(map));



start = [3;3];
goal = [500;500];

closedset = [];
openset = start;
camefrom = [];

g_score(start(1),start(2)) = 1;
f_score(start(1),start(2)) = g_score(start(1),start(2)) + heuristic(start,goal);

% [row,col] = find(f_score==min(f_score(:)))


