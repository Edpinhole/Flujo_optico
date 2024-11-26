%% FIRST TEST
clc
clear 
close all

% traslacion

% the original image is read
FRAME = imread('im_good.jpg');

% scaling down the image
scale = 0.5;
frame = imresize(FRAME, scale);

% creation of 3 new images, one for every channel of the original image RGB
frame_r = frame(:,:,1);
frame_g = frame(:,:,2);
frame_b = frame(:,:,3);

% dimensions of original image
dim = size(frame);
rows = dim(1);
cols = dim(2);

% dimensions of new image (background) - these can be set to be bigger than
% the previos in order to contain the new transforameted imageù
nrows = 1*rows;
ncols = 1*cols;

% definition of the transformation parameters and omogeneus matrix
% in this case dimesions of traslation vector
tx = 10;
ty = 20;
R = [1 0 tx; 0 1 ty;0 0 1];

% starting from the red channel of the original image the red channel of
% the final transformated image is created
newframeR = zeros(nrows, ncols);
for i = 1:rows
    for j = 1:cols
        newind = R*[i,j,1]';
        if newind(1)>0 && newind(1)<nrows && newind(2)>0 && newind(2)<ncols
            newframeR(newind(1),newind(2)) = frame_r(i,j);
        end
    end
end

% starting from the green channel of the original image the green channel 
% of the final transformated image is created
newframeG = zeros(nrows, ncols);
for i = 1:rows
    for j = 1:cols
        newind = R*[i,j,1]';
        if newind(1)>0 && newind(1)<nrows && newind(2)>0 && newind(2)<ncols
            newframeG(newind(1),newind(2)) = frame_g(i,j);
        end
    end
end

% starting from the blue channel of the original image the blue channel of
% the final transformated image is created
newframeB = zeros(nrows, ncols);
for i = 1:rows
    for j = 1:cols
        newind = R*[i,j,1]';
        if newind(1)>0 && newind(1)<nrows && newind(2)>0 && newind(2)<ncols
            newframeB(newind(1),newind(2)) = frame_b(i,j);
        end
    end
end

% the final image is created concatenating the 3 new channel
newframe = uint8(cat(3, newframeR, newframeG, newframeB));

figure
imshow(frame);
figure
imshow(newframe);

figure();
subplot 211
imshow(frame);
im1t = im2double(rgb2gray(frame));
im1 = imresize(im1t, 0.5); % downsize to half

subplot 212
imshow(newframe);
im2t = im2double(rgb2gray(newframe));
im2 = imresize(im2t, 0.5); % downsize to half

% Implementing Lucas Kanade Method
ww = 170;
w = round(ww/2);

% Lucas Kanade Here
% for each point, calculate I_x, I_y, I_t
Ix_m = conv2(im1,[-1 1; -1 1], 'valid'); % partial on x
Iy_m = conv2(im1, [-1 -1; 1 1], 'valid'); % partial on y
It_m = conv2(im1, ones(2), 'valid') + conv2(im2, -ones(2), 'valid'); % partial on t
u = zeros(size(im1));
v = zeros(size(im2));

% within window ww * ww
for i = w+1:size(Ix_m,1)-w
   for j = w+1:size(Ix_m,2)-w
      Ix = Ix_m(i-w:i+w, j-w:j+w);
      Iy = Iy_m(i-w:i+w, j-w:j+w);
      It = It_m(i-w:i+w, j-w:j+w);
      
      Ix = Ix(:);
      Iy = Iy(:);
      b = -It(:); % get b here
    
      A = [Ix Iy]; % get A here
      nu = pinv(A)*b; % get velocity here
      
      u(i,j)=nu(1);
      v(i,j)=nu(2);
   end
end
 
% downsize u and v
u_deci = u(1:10:end, 1:10:end);
v_deci = v(1:10:end, 1:10:end);
% get coordinate for u and v in the original frame
[m, n] = size(im1t);
[X,Y] = meshgrid(1:n, 1:m);
X_deci = X(1:20:end, 1:20:end);
Y_deci = Y(1:20:end, 1:20:end);

% Plot optical flow field
figure();
k = 0.6;
overlapped_photos = (1-k)*frame + k*newframe;
imshow(overlapped_photos);
% imshow(newframe);

hold on;
% draw the velocity vectors
quiver(X_deci, Y_deci, u_deci,v_deci, 'c', 'LineWidth', 1)

%% SECOND TEST
clc
clear 
close all

% the original image is read
FRAME = imread('photo1_flujo.jpg');
FRAME2 = imread('photo2_flujo.jpg');

% scaling down the images
scale = 0.5;
frame = imresize(FRAME, scale);
frame2 = imresize(FRAME2, scale);
figure
imshow(frame);
figure
imshow(frame2);

figure();
subplot 211
imshow(frame);
im1t = im2double(rgb2gray(frame));
im1 = imresize(im1t, 0.5); % downsize to half

subplot 212
imshow(frame2);
im2t = im2double(rgb2gray(frame2));
im2 = imresize(im2t, 0.5); % downsize to half

% Implementing Lucas Kanade Method
ww = 130;
w = round(ww/2);

% Lucas Kanade Here
% for each point, calculate I_x, I_y, I_t
Ix_m = conv2(im1,[-1 1; -1 1], 'valid'); % partial on x
Iy_m = conv2(im1, [-1 -1; 1 1], 'valid'); % partial on y
It_m = conv2(im1, ones(2), 'valid') + conv2(im2, -ones(2), 'valid'); % partial on t
u = zeros(size(im1));
v = zeros(size(im2));

% within window ww * ww
for i = w+1:size(Ix_m,1)-w
   for j = w+1:size(Ix_m,2)-w
      Ix = Ix_m(i-w:i+w, j-w:j+w);
      Iy = Iy_m(i-w:i+w, j-w:j+w);
      It = It_m(i-w:i+w, j-w:j+w);
      
      Ix = Ix(:);
      Iy = Iy(:);
      b = -It(:); % get b here
    
      A = [Ix Iy]; % get A here
      nu = pinv(A)*b; % get velocity here
      
      u(i,j)=nu(1);
      v(i,j)=nu(2);
   end
end
 
% downsize u and v
u_deci = u(1:10:end, 1:10:end);
v_deci = v(1:10:end, 1:10:end);
% get coordinate for u and v in the original frame
[m, n] = size(im1t);
[X,Y] = meshgrid(1:n, 1:m);
X_deci = X(1:20:end, 1:20:end);
Y_deci = Y(1:20:end, 1:20:end);

% Plot optical flow field
figure();
% k = 0.6;
% overlapped_photos = (1-k)*frame + k*frame2;
% imshow(overlapped_photos);
imshow(frame2);

hold on;
% draw the velocity vectors
quiver(X_deci, Y_deci, u_deci,v_deci, 'c', 'LineWidth', 1)
