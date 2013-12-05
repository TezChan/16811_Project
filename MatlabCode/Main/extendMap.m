function extended_map = extendMap(map,radius)

extended_map = map;

%make circle for expansion
X = ones(2*radius+1,1)*[-radius:radius]; 
Y = [-radius:radius]'*ones(1,2*radius+1); 
Z = X.^2 + Y.^2; 
% mesh(Z); 
image = zeros([2*radius+1 2*radius+1]); 
image(find(Z <= radius^2)) = 1; 
image = im2bw(image,.5);
% figure;
% imagesc(image);
% colormap gray;
save('res.mat','image','map');

extended_map = conv2(double(image),double(map));
extended_map = im2bw(extended_map,.5);
extended_map = extended_map(radius+1:end-radius,radius+1:end-radius);
% figure;
% imagesc(extended_map);
% colormap gray;

