%% Find color object and get centroid location
% this takes a picture and finds the color needed for processing
function [x,y] = Imagefindandprocess(color, cam)
% check if there is a valid connection to camera
try
 % take snapshot
img = snapshot(cam);
catch
    warning('No camera connected');
    % default x and y values
    x = 0; 
    y = 0;
    return;
end
try
% check if input color is valid then use
if strcmp(color, 'red')|| strcmp(color, 'blue') || strcmp(color, 'yellow')||strcmp(color, 'green')
    % since value is a color process image
    cen = processImage(img, color);
     if (cen(1) == 0 && cen(1,2) == 0)
         % no more in that color default to 0 0
         y = 0;
         x = 0; 
         return;
     end
    % define m and n value
    m = cen(1);
    n = cen(1,2);        
            else 
                 warning('No color located, defualt to 0 statment');
                 % NEED TO FIND DEFAULT VALUES FOR KNOWING WHEN WE FOULD
                 % ALL COLORS 
                 x = 0;
                 y = 0;
                 return;
end
catch % catch if something happens
    warning('no color value default 0, 0');
    x = 0;
    y = 0;
    return;
end
% get centroid pixel location and convert to x, y cordinates
[x,y] = mn2xy( m, n );
end

% This function takes an RGB image and returns the image with centroid
function centroid = processImage(RGB, color)
% Convert RGB image to chosen color space
I = rgb2hsv(RGB);

if strcmp(color, 'red')
% Define thresholds for channel 1 based on histogram settings
channel1Min = 0.870;
channel1Max = 0.983;

% Define thresholds for channel 2 based on histogram settings
channel2Min = 0.489;
channel2Max = 1.000;

% Define thresholds for channel 3 based on histogram settings
channel3Min = 0.271;
channel3Max = 0.894;

% Create mask based on chosen histogram thresholds
sliderBW = (I(:,:,1) >= channel1Min ) & (I(:,:,1) <= channel1Max) & ...
    (I(:,:,2) >= channel2Min ) & (I(:,:,2) <= channel2Max) & ...
    (I(:,:,3) >= channel3Min ) & (I(:,:,3) <= channel3Max);
BW = sliderBW;

else if strcmp(color, 'blue') 
% Define thresholds for channel 1 based on histogram settings
channel1Min = 0.411;
channel1Max = 0.671;

% Define thresholds for channel 2 based on histogram settings
channel2Min = 0.622;
channel2Max = 1.000;

% Define thresholds for channel 3 based on histogram settings
channel3Min = 0.000;
channel3Max = 1.000;

% Create mask based on chosen histogram thresholds
sliderBW = (I(:,:,1) >= channel1Min ) & (I(:,:,1) <= channel1Max) & ...
    (I(:,:,2) >= channel2Min ) & (I(:,:,2) <= channel2Max) & ...
    (I(:,:,3) >= channel3Min ) & (I(:,:,3) <= channel3Max);
BW = sliderBW;
    else if strcmp(color, 'yellow')
% Define thresholds for channel 1 based on histogram settings
channel1Min = 0.093;
channel1Max = 0.151;

% Define thresholds for channel 2 based on histogram settings
channel2Min = 0.550;
channel2Max = 0.871;

% Define thresholds for channel 3 based on histogram settings
channel3Min = 0.700;
channel3Max = 1.000;

% Create mask based on chosen histogram thresholds
sliderBW = (I(:,:,1) >= channel1Min ) & (I(:,:,1) <= channel1Max) & ...
    (I(:,:,2) >= channel2Min ) & (I(:,:,2) <= channel2Max) & ...
    (I(:,:,3) >= channel3Min ) & (I(:,:,3) <= channel3Max);
BW = sliderBW;

        else % color is green         
% Define thresholds for channel 1 based on histogram settings
channel1Min = 0.244;
channel1Max = 0.359;

% Define thresholds for channel 2 based on histogram settings
channel2Min = 0.855;
channel2Max = 1.000;

% Define thresholds for channel 3 based on histogram settings
channel3Min = 0.257;
channel3Max = 1.000;

% Create mask based on chosen histogram thresholds
sliderBW = (I(:,:,1) >= channel1Min ) & (I(:,:,1) <= channel1Max) & ...
    (I(:,:,2) >= channel2Min ) & (I(:,:,2) <= channel2Max) & ...
    (I(:,:,3) >= channel3Min ) & (I(:,:,3) <= channel3Max);
BW = sliderBW;
        end
    end
end


% Initialize output masked image based on input image.
maskedRGBImage = RGB;

% Set background pixels where BW is false to zero.
maskedRGBImage(repmat(~BW,[1 1 3])) = 0;

% Convert to grayscale image
gray = rgb2gray(maskedRGBImage);
imshow(gray)

% Convert to binary image
binary = imbinarize(gray);
imshow(binary)
% Reduce noise in binary image
binary = bwareaopen(binary, 60);

% Create a binary mask and apply it to the image
% This turns all the pixels outside the ROI (region of interest) black
x = [499 1458 1458 499 499];
y = [457 457 933 933 457];
mask = poly2mask(x,y,1080,1920);
binary(~mask) = 0;

% Calculate centroid, minor and major axis length
properties = regionprops(binary,'Centroid','MajorAxisLength','MinorAxisLength');
% Try to put centroid into matrix to pass back
try
    centroid = properties.Centroid;
% If unable to calculate centroid, default to (0,0) and produce a warning
catch
    warning('No centroid located, defaulting to (1,1)');
    centroid = [0,0];
end
% Calculate the diameter for the object, use diameter to calculate radius
diameters = mean([properties.MajorAxisLength properties.MinorAxisLength],2);
radii = diameters/2;

% % Display
%  imshow(binary);
%  hold on
%  plot(centroid(:,1), centroid(:,2), 'b*')
%  viscircles(centroid,radii);
%  hold off
end

% This function is used to convert an m/n pixel coordinate to an x/y
% position coordinate.  This function centers the x/y coordinate system
% at the robot's end-effector's home position, with +x pointing towards
% the camera, and +y following the RHR.
%
% In addition:  this function assumes that you have successfully
% run the provided "calibrate_camera.m" function prior to its use.
function [ x, y ] = mn2xy( m, n )
%% define calibration distance constants
tot_width_in_cm = 24;
tot_height_in_cm = 10;

%% read in data from xml
xml = xmlread('calibrations/pixels.xml');
xml_pixels = xml.getElementsByTagName('pixel');
pixels = zeros(8,2);
for i = 1:8
   pixels(i, :) = extract_ith_pixel(i-1, xml_pixels);
end

%% organizing data by row.  2nd col *should* be consistent
arm_pixels = pixels(1:3,:);
hole_pixel = mean(pixels(4:5,:));
cam_pixels = pixels(6:8,:);

%% commonly re-used calibration parameter
cam_height_in_pix = mean(cam_pixels(:,2));
arm_height_in_pix = mean(arm_pixels(:,2));
tot_height_in_pix = cam_height_in_pix - arm_height_in_pix;
cam_width_in_pix  = cam_pixels(end,1) - cam_pixels(1,1);
arm_width_in_pix  = arm_pixels(end,1) - arm_pixels(1,1); 

%% calculate x using n
x = (tot_height_in_cm/tot_height_in_pix)*(n - hole_pixel(2));

%% calculate y using m and n
sf_cam = (tot_width_in_cm/cam_width_in_pix); %interpolate between
sf_arm = (tot_width_in_cm/arm_width_in_pix);
percentage = (n-cam_height_in_pix)/(tot_height_in_pix);
sf_cur = percentage * (sf_arm - sf_cam) + sf_cam;
y = sf_cur * (m - hole_pixel(1));
end

% burrows into xml object and rips out numbers
function [pix] = extract_ith_pixel(i, pixels)
    pix(1) = str2num(pixels.item(i).getElementsByTagName('m').item(0).getTextContent());
    pix(2) = str2num(pixels.item(i).getElementsByTagName('n').item(0).getTextContent());
end
