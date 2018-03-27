function [u, v] = LK_hierar(im1, im2, windowSize, u, v)
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Lucas and Kanade of OF method for hierarchial LK
% input
%     im1         Image 1
%     im2         image 2
%     windowSize  size of local neighbourhood for smoothness
%     u, v         optical flow of previosu level
%
% output
%     u,v         velocity of flow in x and y directions
%     You wil get a plot of the same in a figure
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Example
% LK_hierar(im1, im2, 5, uold, vold);

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

% compute Iy
Iy1 = conv2(im1, maskY,'same');
Iy2 = conv2(im2, maskY,'same');

% compute It
It1 = conv2(im1, maskT1,'same');
It2 = conv2(im2, maskT2, 'same');

% get the floor and ceil from window/2 to use for indexes for window
temp = floor(windowSize/2);
temp1 = ceil(windowSize/2);

%create meshgrid and add u and v to get updated positions
[rm, cm] = meshgrid(1:size(im2,2),1:size(im2,1));
Yrow = rm + u;
Ycol = cm + v;

% to get the shifted window values (with u and v vectors) for image2
for i = 1:size(Ix2,1)
    for j = 1:size(Ix2,2)
        row = round(Ycol(i,j));
        col = round(Yrow(i,j));
        
        % if pixel indices < 1 or greater than image size then skip
        if row < 1 | col < 1 | row > size(Ix2,1) | col > size(Ix2,2)
            Ix2new(i,j) = Ix2(i,j);
            Iy2new(i,j) = Iy2(i,j);
            It2new(i,j) = It2(i,j);
        else
            Ix2new(i,j) = Ix2(row, col);
            Iy2new(i,j) = Iy2(row, col);
            It2new(i,j) = It2(row, col);
        end
    end
end

% compute the gradients along x,y and t for two images
Ix = (Ix1 + Ix2new)/2;
Iy = (Iy1 + Iy2new)/2;
It = (It1 + It2new);

% get the optical flow for each pixel defined by i and j
for i = temp1:size(im1,1)-temp1
    for j = temp1:size(im1,2)-temp1
        
        % define a window
        Rrow = i-temp;
        Rrow1 = i+temp;
        Rcol = j-temp;
        Rcol1 = j+temp;
        
        %                 % window for first image
        %                 Ix1new = Ix1(Rrow:Rrow1, Rcol:Rcol1);
        %                 Iy1new = Iy1(Rrow:Rrow1, Rcol:Rcol1);
        %                 It1new = It1(Rrow:Rrow1, Rcol:Rcol1);
        %
        %                 % window 2
        %                 YrowC = round(Rrow + Yrow(i,j));
        %                 Yrow1C = round(Rrow1 + Yrow(i,j));
        %                 YcolC = round(Rcol + Ycol(i,j));
        %                 Ycol1C = round(Rcol1 + Ycol(i,j));
        %
        %                 if (YrowC < 1 | Yrow1C < 1 | YcolC < 1 | Ycol1C < 1 | YrowC > size(im1,1) | Yrow1C > size(im1,1) | YcolC > size(im1,2) | Ycol1C > size(im1,2))
        %                     %         Ix2new = Ix2(Rrow:Rrow1, Rcol:Rcol1);
        %                     %         Iy2new = Iy2(Rrow:Rrow1, Rcol:Rcol1);
        %                     %         It2new = It2(Rrow:Rrow1, Rcol:Rcol1);
        %                 else
        %                     % window with sifyter u and v for second image
        %                     Ix2new = Ix2(YrowC:Yrow1C, YcolC:Ycol1C);
        %                     Iy2new = Iy2(YrowC:Yrow1C, YcolC:Ycol1C);
        %                     It2new = It2(YrowC:Yrow1C, YcolC:Ycol1C);
        %
        %                 end
        
        % get the values in neighbourhood defined by window
        Ixnew = Ix(Rrow:Rrow1, Rcol:Rcol1);
        Iynew = Iy(Rrow:Rrow1, Rcol:Rcol1);
        Itnew = It(Rrow:Rrow1, Rcol:Rcol1);
        
        %                 % get Ix, Iy, It
        %                 Ixnew = (Ix1new + Ix2new)/2;
        %                 Iynew = (Iy1new + Iy2new)/2;
        %                 Itnew = It1new + It2new ;
        
        % get the A and B matrices to solve Ax = B
        A = [Ixnew(:) Iynew(:)];
        B = -1 * Itnew(:);
        
        % solve Ax = B
        data = pinv(A) * B;

        % store the OF for pixel in u and v
        u(i,j) = data(1,:);
        v(i,j) = data(2,:);
        
    end
end


u(isnan(u))=0;
v(isnan(v))=0;
