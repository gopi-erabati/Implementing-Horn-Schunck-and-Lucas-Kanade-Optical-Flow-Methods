function [u, v] = LK(im1, im2, windowSize)
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Lucas and Kanade of OF method
% input
%     im1         Image 1
%     im2         image 2
%     windowSize  size of window for local neighbourhood
%
% output
%     u,v         velocity of flow in x and y directions
%     You wil get a plot of the same in a figure
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Example
% LK(im1, im2, 3);

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

% variables to store values to deefine window later
temp = floor(windowSize/2);
temp1 = ceil(windowSize/2);

%intialise u and v
u = zeros(size(im1));
v = zeros(size(im1));

for i = temp1:size(im1,1)-temp1
    for j = temp1:size(im1,2)-temp1
        
        % get values in local window of pixel
        Ixnew = Ix(i-temp:i+temp, j-temp:j+temp);
        Iynew = Iy(i-temp:i+temp, j-temp:j+temp);
        Itnew = It(i-temp:i+temp, j-temp:j+temp);
        
        % compute A and B matrix
        A = [Ixnew(:) Iynew(:)];
        B = -1 * Itnew(:);
        
        % get the solution to Ax = B
        data = pinv(A) * B;
        
        % store velocity in u and v at pixel locations i and j
        u(i,j) = data(1,:);
        v(i,j) = data(2,:);
        
    end
end

%plot the flow
plotOF_arrows(u, v);

u(isnan(u))=0;
v(isnan(v))=0;
