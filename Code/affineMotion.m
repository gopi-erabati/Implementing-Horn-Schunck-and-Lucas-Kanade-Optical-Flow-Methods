function [u,v] = affineMotion( im1, im2 )
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % function to get affine parameters
% input
%     im1         Image 1
%     im2         image 2
%
% output
%     u,v         velocity of flow in x and y directions
%     You wil get a plot of the same in a figure
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Example
% affineMotion( im1, im2 );

%convert rgb to gray
if size(size(im1),2)==3
    im1=rgb2gray(im1);
end
if size(size(im2),2)==3
    im2=rgb2gray(im2);
end

%convert image to double
im1=double(im1);
im2=double(im2);

%apply gaussain filter to smooth
im1 = imgaussfilt(im1,2);
im2 = imgaussfilt(im2,2);

% mask for X derivative
maskX = [-1 1
    -1 1];

% mask for Y Derivative
maskY = [-1 -1
    1 1];

% mask for time derivative
maskT1 =  ones(2,2);
maskT2 = -1 * ones(2,2);

% Compute Ix
Ix1 = conv2(im1, maskX, 'same');
Ix2 = conv2(im2, maskX, 'same');
Ix = (Ix1 + Ix2)./2;

% compute Iy
Iy1 = conv2(im1, maskY,'same');
Iy2 = conv2(im2, maskY,'same');
Iy = (Iy1 + Iy2)./2;

% compute It
It1 = conv2(im1, maskT1,'same');
It2 = conv2(im2, maskT2, 'same');
It = (It1 + It2);

% intialise M and P matrices to speed up the process
M = zeros(numel(im1), 6);
P = zeros(numel(im1), 6);

%compute M and P matrices
rown = 1; % rwo counter
for i = 1 : size(im1,1)
    for j = 1 : size(im1,2)
        
        M(rown,:) = [ Ix(i,j)*j, Ix(i,j)*i, Ix(i,j), Iy(i,j)*j, Iy(i,j)*i, Iy(i,j)];
        P(rown,:) = [ -1*It(i,j)];
        
        rown = rown + 1; % increment the counter
    end
end

%calculate the theta using pusedo inverse
theta = M\ P;

% calculate u and v by
%u = ax + by + c
%v = dx + ey + f
[xm, ym] = meshgrid(1:size(im2,2), 1:size(im2,1));
u = theta(1) *xm + theta(2) * ym + theta(3);
v = theta(4) * xm + theta(5) * ym + theta(6);

% uncomment to show plot of OF
% plotOF_arrows(u, v);
