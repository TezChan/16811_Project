% Neil Abcouwer and Priya Deo
% 16-811 Project
% 11/14/2013
%
% Main file for A star

clear all
close all
clc

radius = 15;
robot_dist = 45;
rot = true;


%get map
map = imread('map4.png');
map = im2bw(map,.5);
map = ~map;
figure
imagesc(map)
colormap gray
extended_map = extendMap(map,radius);

compare = map+extended_map;
figure
imagesc(compare)
colormap gray

start_x = 5; start_y = 5; start_th = 0;
goal_x = size(extended_map,2)-20; goal_y = size(extended_map,1)-20; goal_th = -90;

% choose sampling points
res = 20;
th_res = 30;
if rot
    [X,Y,TH] = meshgrid(res:res:size(extended_map,2),...
        res:res:size(extended_map,1),...
        -180+th_res:th_res:180);
else
    [X,Y] = meshgrid(res:res:size(extended_map,2),...
        res:res:size(extended_map,1));
end

% for every pt
X = [start_x goal_x X(1:end)];
Y = [start_y goal_y Y(1:end)];
if rot
    TH = [start_th goal_th TH(1:end)];
    Y2 = round(Y+sind(TH)*robot_dist);
    X2 = round(X+cosd(TH)*robot_dist);
end
OKlong = ones(size(X));

if rot
    for ii=1:length(X)
        if (extended_map(Y(ii),X(ii)) ...
                || Y2(ii)<=0 ...
                || X2(ii)<=0 ...
                || Y2(ii)>size(extended_map,1) ...
                || X2(ii)>size(extended_map,2) ...
                || extended_map(Y2(ii),X2(ii)))
            OKlong(ii)=0;
        end
    end
else
    for ii=1:length(X)
        % check if on obstacle
        if extended_map(Y(ii),X(ii))
            OKlong(ii)=0;
        end
    end
end

X = X(OKlong>0);
Y = Y(OKlong>0);
if rot
    TH = TH(OKlong>0);
    X2 = X2(OKlong>0);
    Y2 = Y2(OKlong>0);
end

hold on
% plot(X,Y,'r.')

% graph = zeros(length(X));
aa = zeros(size(X));
bb = zeros(size(X));
cc = zeros(size(X));
h_list = inf(length(X),1);
display(strcat('populating graph of ',' ',int2str(length(X)),' nodes'))
timer1 = tic();
last_time = 0;
est_time = true;
kk = 1;
for ii = 1:length(X)
    if(toc(timer1)-last_time>10)
        last_time = last_time+10;
        time_proj = (length(X)-ii)/ii *last_time;
        display(strcat('estimated completion in ',' ', num2str(time_proj),' seconds'));
    end
    for jj = ii:length(X)
        if rot
            h_list(ii) = norm([(X(ii)+X2(ii))/2-goal_x;
                (Y(ii)+Y2(ii))/2-goal_y]);
            dist = norm([X(ii)-X(jj);
                Y(ii)-Y(jj);
                TH(ii)-TH(jj)]);
        else
            h_list(ii) = norm([X(ii)-goal_x;
                Y(ii)-goal_y]);
            dist = norm([X(ii)-X(jj);
                Y(ii)-Y(jj)]);
        end
        %         h_list(ii) = norm([X(ii)-goal_x;
        %             Y(ii)-goal_y]);
        %         dist = norm([X(ii)-X(jj);
        %             Y(ii)-Y(jj)]);
        
        %         if dist<=sqrt(3)*res
        if dist<=sqrt(3)*res
            %             graph(ii,jj) = dist;
            %             graph(jj,ii) = dist;
            aa(kk) = ii;
            bb(kk) = jj;
            cc(kk) = dist;
            kk = kk+1;
            aa(kk) = jj;
            bb(kk) = ii;
            cc(kk) = dist;
            kk = kk+1;
        end
    end
end

aa = aa(aa~=0);
bb = bb(aa~=0);
cc = cc(aa~=0);
graph = sparse(aa,bb,cc);



display('starting astar')

[pathdist, path, pred]=aStar(graph,h_list,1,2);

xpath = X(path);
ypath = Y(path);
if rot
    x2path = X2(path);
    y2path = Y2(path);
end

hold on
plot(xpath,ypath,'r:')
plot(xpath,ypath,'r*')
if rot
    plot(x2path,y2path,'ro')
    line([xpath; x2path],[ypath; y2path],'Color','r')
end


% %remove points on obstacles
% % for ii =
%
%
%
%
% g_score = NaN(size(map));
% f_score = NaN(size(map));
% openset = zeros(size(map));
% closedset = zeros(size(map));
% camefrom = zeros(size(map));
%
%
%
% start = [3;3];
% goal = [500;500];
%
% closedset = [];
% openset = start;
% camefrom = [];
%
% g_score(start(1),start(2)) = 1;
% f_score(start(1),start(2)) = g_score(start(1),start(2)) + heuristic(start,goal);
%
% % [row,col] = find(f_score==min(f_score(:)))
%
%
