% This function takes an RGB image and returns the image with centroid
function centroid = processImage(RGB)
% Crop the image to just the workspace
%RGB = imcrop(RGB, [470 258 1020 822]);

% Convert RGB image to chosen color space
I = rgb2hsv(RGB);

% Define thresholds for channel 1 based on histogram settings
channel1Min = 0.969;
channel1Max = 0.074;

% Define thresholds for channel 2 based on histogram settings
channel2Min = 0.000;
channel2Max = 1.000;

% Define thresholds for channel 3 based on histogram settings
channel3Min = 0.000;
channel3Max = 1.000;

% Create mask based on chosen histogram thresholds
sliderBW = ( (I(:,:,1) >= channel1Min) | (I(:,:,1) <= channel1Max) ) & ...
    (I(:,:,2) >= channel2Min ) & (I(:,:,2) <= channel2Max) & ...
    (I(:,:,3) >= channel3Min ) & (I(:,:,3) <= channel3Max);
BW = sliderBW;

% Initialize output masked image based on input image.
maskedRGBImage = RGB;

% Set background pixels where BW is false to zero.
maskedRGBImage(repmat(~BW,[1 1 3])) = 0;

% Convert to grayscale image
gray = rgb2gray(maskedRGBImage);

% Convert to binary image
binary = imbinarize(gray);

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
% If unable to calculate centroid, default to (1,1) and produce a warning
catch
    warning('No centroid located, defaulting to (1,1)');
    centroid = [1,1];
end
% Calculate the diameter for the object, use diameter to calculate radius
diameters = mean([properties.MajorAxisLength properties.MinorAxisLength],2);
radii = diameters/2;

% % Display
% imshow(binary);
% hold on
% plot(centroids(:,1), centroids(:,2), 'b*')
% viscircles(centroids,radii);
% hold off
end