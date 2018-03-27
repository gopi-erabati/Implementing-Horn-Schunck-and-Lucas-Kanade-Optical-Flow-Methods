function [ output_args ] = compensateCamera(  )
%Compensate camera motion
% using Optical flow between image frames

%choose path of files
path = uigetdir('cwd');

% directory of path to acces files
files = dir2(path);

%read images and put them in 3D stack
for i = 1: length(files)
    img(:,:,i) = imread([path,'\',files(i).name]);
end

% copy first image to compesnated sequence
imgComp(:,:,1) = img(:,:,1);

%initialise u and v with zeros to accumulate
unew = zeros(size(img(:,:,1)));
vnew = zeros(size(img(:,:,1)));

% run for all images
for i = 1 : length(files)-1
    
    % get optical flow for image sequences
    [u,v] = affineMotion(img(:,:,i), img(:,:,i+1));
    
    % accumulate the flow
    unew = unew + u;
    vnew = vnew + v;
    
    % intialise compensation with zeros
    imgComp(:,:,i+1) = zeros(size(img,1),size(img,2));
    
    % for all pixels get the intensity of next image at shifted position
    % with optical flow u and v
    for j = 1: size(img,1)
        for k = 1:size(img,2)
            row = round(j + vnew(j,k));
            col = round(k + unew(j,k));
            
            % check for valid pixel indices
            if row < 1 | col < 1| row > size(img,1) | col > size(img,2)
            else
                imgComp(j,k,i+1) = img(row, col,i+1);
            end
        end
    end
    
    % show the frames
    imshow(imgComp(:,:,i+1),[]);
    pause(0.2);
end



