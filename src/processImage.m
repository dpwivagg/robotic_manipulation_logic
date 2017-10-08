% This function takes an RGB image and returns the image with centroid
function centroid = processImage(RGB)
    % Convert RGB image to chosen color space
    I = rgb2hsv(RGB);

    % Define thresholds for HSV -> Blue
    channel1Min(1) = 0.389;
    channel1Max(1) = 0.538;
    channel2Min(1) = 0.867;
    channel2Max(1) = 1.000;
    channel3Min(1) = 0.045;
    channel3Max(1) = 1.000;

    % Define thresholds for HSV -> Green
    channel1Min(2) = 0.248;
    channel1Max(2) = 0.297;
    channel2Min(2) = 0.867;
    channel2Max(2) = 1.000;
    channel3Min(2) = 0.250;
    channel3Max(2) = 0.779;

    % Define thresholds for HSV -> Yellow
    channel1Min(3) = 0.096;
    channel1Max(3) = 0.118;
    channel2Min(3) = 0.585;
    channel2Max(3) = 1.000;
    channel3Min(3) = 0.601;
    channel3Max(3) = 1.000;
    
    % Preallocate array for centroids
    centroid = [0 0;...
                0 0;...
                0 0];
    
    % For each of the color parameters, create a binary image
    for i = 1:3
        % Create mask based on chosen histogram thresholds
        sliderBW = (I(:,:,1) >= channel1Min(i) ) & (I(:,:,1) <= channel1Max(i)) & ...
        (I(:,:,2) >= channel2Min(i) ) & (I(:,:,2) <= channel2Max(i)) & ...
        (I(:,:,3) >= channel3Min(i) ) & (I(:,:,3) <= channel3Max(i));
        BW = sliderBW;
        
        % Initialize output masked image based on input image.
        maskedRGBImage = RGB;

        % Set background pixels where BW is false to zero.
        maskedRGBImage(repmat(~BW,[1 1 3])) = 0;
        
        % Convert to grayscale image
        gray = rgb2gray(maskedRGBImage);

        % Convert to binary image
        binary = imbinarize(gray);
        
        % Create a binary mask and apply it to the image
        % This turns all the pixels outside the ROI (region of interest) black
        x = [499 1458 1458 499 499];
        y = [457 457 933 933 457];
        mask = poly2mask(x,y,1080,1920);
        binary(~mask) = 0;
        
        % Reduce noise in the binary image by eliminating objects with less
        % than 60 pixels
        binary = bwareaopen(binary,60);
        
        % Calculate centroid, minor and major axis length
        properties = regionprops(binary,'Centroid');
        % Try to put centroid into matrix to pass back
        try
            centroid(i,:) = properties.Centroid;
        % If unable to calculate centroid, default to (1,1) and produce a warning
        catch
            warning('No centroid located, defaulting to (1,1)');
            centroid(i,:) = [1,1];
        end
        
        centroid(i,:) = mn2xy(centroid(i,1),centroid(i,2));
        
        % Display
        imshow(RGB);
        hold on
        plot(centroid(:,1), centroid(:,2), 'b*')
        hold off
    end
end