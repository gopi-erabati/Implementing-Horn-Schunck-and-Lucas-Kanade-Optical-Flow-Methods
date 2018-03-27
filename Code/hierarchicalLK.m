function [u,v] = hierarchicalLK(im1, im2, numLevels, windowSize)
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Hierarchical implementation of Lucas and Kanade of OF method
% input
%     im1               Image 1
%     im2               image 2
%     numLevels         Levels of pyramid
%     windowSize        local neighbourhood window size
%
% output
%     u,v         velocity of flow in x and y directions
%     You wil get a plot of the same in a figure
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Example
% hierarchicalLK(im1, im2, 5, 5);

% graylevel image
if (size(im1,3) ~= 1) || (size(im2, 3) ~= 1)
    im1 = rgb2gray(im1);
    im2 = rgb2gray(im2);
end;

%convert image to double
im1 = double(im1);
im2 = double(im2);

%apply gaussain filter to smooth
im1 = imgaussfilt(im1,2);
im2 = imgaussfilt(im2,2);

% Build the pyramids
im1Pyr = {};
im2Pyr = {};
im1Pyr{1} = im1;
im2Pyr{1} = im2;

for i=2:numLevels
    im1Pyr{i} = imresize(im1Pyr{i-1},1/2);
    im2Pyr{i} = imresize(im2Pyr{i-1},1/2);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Begin of "Write your code here" section %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%apply LK for highest level
[uold, vold] = LK(im1Pyr{numLevels}, im2Pyr{numLevels}, windowSize);
close all; % to close the plot created by LK

for i = numLevels-1 : -1 : 1
    
    % bilinear interpolate to double dimension
    u = imresize(uold, 2*size(uold), 'bilinear');
    v = imresize(vold, 2*size(vold), 'bilinear');
    
    %multiply by 2 (magnitude)
    uold = 2 * u;
    vold = 2 * v;
    
    % apply LK to next higher level
    [u,v] = LK_hierar(im1Pyr{i}, im2Pyr{i}, windowSize, uold, vold);
    
    % add the velocity flow
    unew = uold + u;
    vnew = vold + v;
    
    %update the velocity flow
    uold = unew;
    vold = vnew;
end

% plot the optical flow
plotOF_arrows(uold, vold);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% End of "Write your code here" section %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
