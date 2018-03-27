function [u, v] = HS(im1, im2, lambda, ite)
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Horn-Schunck optical flow method
% input
%     im1         Image 1
%     im2         image 2
%     lambda      smoothness factor
%     ite         number of iterations
%
% output
%     u,v         velocity of flow in x and y directions
%     You wil get a plot of the same in a figure
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Example
% HS(im1, im2, 20, 200);

% convert rgb to gray
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

% Compute Ix (gradient along x)
Ix1 = conv2(im1, maskX, 'same');
Ix2 = conv2(im2, maskX, 'same');
Ix = (Ix1 + Ix2)./2;

% compute Iy (gradient along Y)
Iy1 = conv2(im1, maskY,'same');
Iy2 = conv2(im2, maskY,'same');
Iy = (Iy1 + Iy2)./2;

% compute It (gradient along time)
It1 = conv2(im1, maskT1,'same');
It2 = conv2(im2, maskT2, 'same');
It = (It1 + It2)./2;

% intialise to zeros
uPrevs = zeros(size(im1));
vPrevs = zeros(size(im1));

% construct a lamba matrix
lambdaMat = lambda * ones(size(im1));

for iter = 1:ite
    %construct a mask to average except central pixel
    avgMask = [1/12 1/6 1/12
        1/6 0 1/6
        1/12 1/6 1/12];
    
    
    % calculate u and v around neighbourhood
    uPrevs = conv2(uPrevs, avgMask, 'same');
    vPrevs = conv2(vPrevs, avgMask, 'same');
    
    % calculate u and v
    u = uPrevs - (Ix .* (Ix .* uPrevs + Iy .* vPrevs + It)./(lambdaMat.^2 + Ix.^2 + Iy.^2));
    v = vPrevs - (Iy .* (Ix .* uPrevs + Iy .* vPrevs + It)./(lambdaMat.^2 + Ix.^2 + Iy.^2));
    
    %update the u and v
    uPrevs = u;
    vPrevs = v;
    
end

% plot the OF
plotOF_arrows(u, v)

u(isnan(u))=0;
v(isnan(v))=0;
