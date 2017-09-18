% This function takes an RGB image and returns the image with centroid
function processImage(RGB)
% Crop the image to just the workspace
RGB = imcrop(RGB, [470 258 1020 822]);

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

% Calculate centroid, minor and major axis length
properties = regionprops(binary,'Centroid','MajorAxisLength','MinorAxisLength');
% Put all centroids in one matrix
centroids = properties.Centroid;
% Calculate the diameter for the object, use diameter to calculate radius
diameters = mean([properties.MajorAxisLength properties.MinorAxisLength],2);
radii = diameters/2;

% Display
imshow(binary)
hold on
plot(centroids(:,1), centroids(:,2), 'b*')
viscircles(centroids,radii);
hold off
end