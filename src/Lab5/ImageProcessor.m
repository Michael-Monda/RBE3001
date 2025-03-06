classdef ImageProcessor < handle
    properties 
        %treat these as if they were c++ global vars. 
        camera;  % The Camera object for this image processor
        cameraID = 2;
        mask;
        % What other attributes does an ImageProcessor need?
        Params;     %camera params post calibration
        Intrinsic;  %intrinsic camera info
        Rotation;   %rotation mat from the camera's frame to the grid frame
        Translation;    %translation vector from camera to grid origin
        %transformation matrix found from the base of the robot to the
        %board's reference 
        robot_camera_transform = [0, 1, 0, 85;
                                  1, 0, 0, -110;
                                  0, 0, -1, 0;
                                  0, 0, 0, 1];
        
        %these are required for the adjustment of the target position.
        altitude = 179;  %mm
        radius = 12.5;    %radius of a target ball
        gamma;  %arctan((alt/measured location) || (radius/change in xy)) 
        debug = true;
        binmask;
        validCentroids = [];

    end

    methods
        % @@@@@@
        % README: nvargs
        % @@@@@@
        % (I ain't reading all that)
        %nvargs are basically just constants unique to the function they
        %are instantiated in.

        function self = ImageProcessor(nvargs)
            %% this function creates an object instance of ImageProcessor.
            %In doing that, it creates a camera object and calibrates it.
            %There is therefore no need to reference the
            %collaborationsession.mat file from signoff 1, however we
            %should add the workspace containing Camera.m to the path just
            %in case.
            arguments
                nvargs.debug logical = true;
            end
            addpath '/home/jahowhannesian/Documents/rbe3001/RBE3001_C25_Team_16';
            self.camera = Camera();  % Instantiate a Camera obj
            self.mask = ColorMask(); % Instantiate a Color Masker obj

            %get the camera position and extract the relevant info.
            %Built-in MATLAB methods for this are not working for some
            %reason.
            self.Rotation = self.camera.cam_R;
            self.Translation = self.camera.cam_T;
            self.Intrinsic = self.camera.cam_IS;
            %now pointsToWorld can be used properly by calling:
            %pointsToWorld(self.Intrinsic, self.Rotation, self.Translation, imageCoordinates).
            self.binmask = self.generate_static_mask();
        end

        function mask = generate_static_mask(self, nvargs)
            %% This function produces a binary static mask.
            %depending upon the size of the detected checkerboard. Its 
            %output can be element-wise multiplied by an image to 
            %effectively crop everything null, and can also be multiplied 
            %by another binary filter to "combine the crops."
            arguments
                self ImageProcessor;
                nvargs.margin double = 45; % px
                nvargs.trap double = 60;
            end

            img = self.camera.getimage();
            [corners, ~] = detectCheckerboardPoints(img);
            %loosely establish the vertices of the polygon we want to keep.
            xmin = min(corners(:,1)) - nvargs.margin;
            ymax = max(corners(:,2)) + nvargs.margin;
            ymin = min(corners(:,2)) - nvargs.margin/1.45;
            xmax = max(corners(:,1)) + nvargs.margin;
            %firmly establish the vertices of the polygon we want to keep.
            NW_corner = [corners(1, 1)-nvargs.margin + 10, ymin];
            NE_corner = [corners(35, 1)+nvargs.margin, ymin];
            SE_corner = [xmax+nvargs.trap-15, ymax-15];
            SW_corner = [xmin-nvargs.trap+5, ymax]; 
            %there is 100% a more intelligent way to do this than just 
            %creating a"close enough trapezoid," but I am not that
            %intelligent lmao

            [imgHeight, imgWidth, ~] = size(img);
            polyx = [NW_corner(1), NE_corner(1), SE_corner(1), SW_corner(1)];
            polyy = [NW_corner(2), NE_corner(2), SE_corner(2), SW_corner(2)];

            mask = poly2mask(polyx, polyy, imgHeight, imgWidth);
            
            % Display masked image if debugging
            if self.debug
                maskedImg = img .* uint8(mask);
                imshow(maskedImg);
            end
        end

        function p_robot = image_to_robot(self, uvpos)
            %% This function produces robot-frame x-y coords from an image.
            %pretty self explanatory. This works by using a transformation
            %matrix to shift the frame of reference from the origin on the
            %checkerboard to the global origin at the base of the robot
            %(which we base all the movements off of in the code).

            %convert pixel measurement to coordinates in the checkerboard's
            %reference frame (checkpos):
            checkpos = pointsToWorld(self.Intrinsic, self.Rotation, self.Translation, uvpos);
            %convert checkpos to task space position
            checkerboard_pos = [checkpos, 0, 1]';
            taskspace_pos = self.robot_camera_transform * checkerboard_pos;
            p_robot = taskspace_pos(1:2)';
        end

        function uv_centroids = detect_centroids(self, nvargs)
            %% This function detects the centroids of objects in frame.
            %it does this by creating separate masks by color, and then
            %applying another mask to all objects below a given area in
            %pixels. The intended output of this function is an nx3 matrix
            %of n centroids by position [x, y] and color enum [1, 2, ...];
            arguments
                self ImageProcessor;
                nvargs.min_size double = 700;  % chooose a value
            end
            output = zeros(20, 3);

            %get image
            newImage = self.camera.getimage();

            %mask off everything except colors of interest.
            blue = self.mask.blue(newImage);   %for marker cap (extra credit!)
            green = self.mask.green(newImage);
            yellow = self.mask.yellow(newImage);
            orange = self.mask.orange(newImage);
            red = self.mask.red(newImage);

            %remove everything smaller than a minimum area established in
            %nvargs.
            blues = bwareaopen(blue, nvargs.min_size);
            greens = bwareaopen(green, nvargs.min_size);
            yellows = bwareaopen(yellow, nvargs.min_size);
            oranges = bwareaopen(orange, nvargs.min_size);
            reds = bwareaopen(red, nvargs.min_size);

            %combine these masks with the binary mask to get the images for
            %analysis, and count the number of remaining entities using 
            %bwlabel.
            ref_blues = bwlabel(blues .* self.binmask);
            ref_greens = bwlabel(greens .* self.binmask);
            ref_yellows = bwlabel(yellows .* self.binmask);
            ref_oranges = bwlabel(oranges .* self.binmask);
            ref_reds = bwlabel(reds .* self.binmask);
            
            %now identify centroid locations.
            blue_centroids = regionprops(ref_blues, 'Centroid');
            green_centroids = regionprops(ref_greens, 'Centroid');
            yellow_centroids = regionprops(ref_yellows, 'Centroid');
            orange_centroids = regionprops(ref_oranges, 'Centroid');
            red_centroids = regionprops(ref_reds, 'Centroid');
            %yeah the compiler doesn't like the use of bwlabel, but it can
            %cope.

            %plot these to check if the centroids were detected and  ID'd 
            %properly.
            if self.debug
                imshow(newImage .* uint8(self.binmask));
                hold on;
                
                for k = 1:numel(blue_centroids)
                    %specifiy which centroid we want, and pull the position
                    x_blue = blue_centroids(k).Centroid(1,1);
                    y_blue = blue_centroids(k).Centroid(1,2);
                    text(x_blue, y_blue, sprintf('blue'), ...
                    'Color', 'red', 'FontSize', 12, 'FontWeight', 'bold');
                end
                
                for k = 1:numel(green_centroids) 
                    %specifiy which centroid we want, and pull the position
                    x_green = green_centroids(k).Centroid(1,1);
                    y_green = green_centroids(k).Centroid(1,2);
                    text(x_green, y_green, sprintf('green'), ...
                    'Color', 'red', 'FontSize', 12, 'FontWeight', 'bold');
                end
                   
                for k = 1:size(yellow_centroids) %not supposed to use size but it works so don't touch it pls
                    %specifiy which centroid we want, and pull the position
                    x_yellow = yellow_centroids(k).Centroid(1,1);
                    y_yellow = yellow_centroids(k).Centroid(1,2);
                    text(x_yellow, y_yellow, sprintf('yellow'), ...
                    'Color', 'red', 'FontSize', 12, 'FontWeight', 'bold');
                end
                
                for k = 1:size(orange_centroids) %not supposed to use size but it works so don't touch it pls
                    %specifiy which centroid we want, and pull the position
                    x_orange = orange_centroids(k).Centroid(1,1);
                    y_orange = orange_centroids(k).Centroid(1,2);
                    text(x_orange, y_orange, sprintf('orange'), ...
                    'Color', 'red', 'FontSize', 12, 'FontWeight', 'bold');
                end
                
                for k = 1:size(red_centroids)
                    %specifiy which centroid we want, and pull the position
                    % disp(k);
                    x_red= red_centroids(k).Centroid(1,1);
                    y_red= red_centroids(k).Centroid(1,2);
                    text(x_red, y_red, sprintf('red'), ...
                    'Color', 'red', 'FontSize', 12, 'FontWeight', 'bold');
                end

                hold off;
            end
            %need to package all centroids into a single nx3 matrix so it
            %can be exported into the "correct centroids" func.

            %we start by decomposing the Centroid struct into nx2 matrices,
            %and feeding those into a blank 15x2 matrix (15 elements should
            %be long enough, right??????????)
            footprint = numel(red_centroids);
            j = footprint; %pull length of list 
            for i = 1:footprint
                output(i, :) = [1, red_centroids(i).Centroid(1, :)];
            end
            footprint = numel(orange_centroids);
            k = j + footprint;
            for i = 1:footprint
                output((i+k), :) = [2, orange_centroids(i).Centroid(1, :)]; 
            end
            footprint = numel(yellow_centroids);
            l = k + footprint;
            for i = 1:footprint
                output((i+l), :) = [3, yellow_centroids(i).Centroid]; 
            end
            footprint = numel(green_centroids);
            m = l + footprint;
            for i = 1:footprint
                output((i+m), :) = [4, green_centroids(i).Centroid]; 
            end
            footprint = numel(blue_centroids);
            n = m + footprint;
            for i = 1:footprint
                output((i+n), :) = [5, blue_centroids(i).Centroid]; 
            end

            uv_centroids = output(:, :);    %output the matrix of centroids 
                                            %and colors
        end

        
        function true_centroids = correct_centroids(self, input_centroids, nvargs)
            %% This adjusts detected centroids to the true position.
            %by intaking the matrix of detected centroids and associated
            %colors, we can extract each row and adjust the x and y values
            %accoring to an algorithm we write using the height of the
            %camera. Additionally, the outputted matrix should have an
            %additional fourth column of z values, which are constant
            %because they are equivalent to the radius of the ball.
            arguments
                self ImageProcessor;
                input_centroids double;
                nvargs.ball_z = self.radius + 5; % testing  
            end
            
            positions = zeros(20, 2);
            appendable_matrix = zeros(20, 3);
            unprocessed_output = zeros(20, 4);
            true_centroids = zeros(20, 4);
            

            %ideally, we would be able to initialize the matrices to
            %contain the same number of rows as the output.
            positions(:, 1:2) = input_centroids(:, 2:3);
            
            m = size(input_centroids);
            n = m(1);

            %adjust each centroid coord using inverse kinematics
            for i = 1:n
                if positions(i, 1) ~= 0
                    pixel_x = positions(i, 1);
                    pixel_y = positions(i, 2);
                    true_xy = self.image_to_robot([pixel_x, pixel_y]);
                    x = true_xy(1, 1); % testing
                    disp("x = ");
                    disp(x);
                    y = true_xy(1, 2); % testing
                    disp("y = ");
                    disp(y);
                    %theta = atan2d(y, x);
                    %y = y - y*sin(theta);
                    %x = x - x*cos(theta);
                
                    psi = atan2d(pixel_y, (546 - pixel_x));
                    campos = [370, 0, 179];
                    measuredpos = [x, y];
                    r = sqrt(((campos(1) - measuredpos(1))^2) + (y^2)) ;
                    
                    self.gamma = atan2d(self.altitude,r);
                    delta_xy = (self.radius / tand(self.gamma)); %was +10
                    %from the POV of the robot towards the camera...
                    if y>=-75 && x>=125 %top left
                        loss_y = delta_xy*sind(psi);
                        loss_x = delta_xy*cosd(psi) ;
                        appendable_matrix(i, :) = [x-1.1*loss_x + 12, y-loss_y + y*0.1, nvargs.ball_z];
                    elseif y>=-75 && x<=125 %bottom left
                        loss_y = delta_xy*sind(psi);
                        loss_x = delta_xy*cosd(psi) ;
                        appendable_matrix(i, :) = [x-loss_x + 2, y-0.3*loss_y, nvargs.ball_z];
                    elseif y<=-75 && x<=125 %bottom right
                        loss_y = delta_xy*sind(psi);
                        loss_x = delta_xy*cosd(psi) ;
                        appendable_matrix(i, :) = [x-loss_x + 18, y-0.3*loss_y, nvargs.ball_z];
                    elseif y<=-75 && x>=125 %top right
                        loss_y = delta_xy*sind(psi);
                        loss_x = delta_xy*cosd(psi) ;
                        appendable_matrix(i, :) = [x-loss_x+20, y-0.3*loss_y - y*0.2, nvargs.ball_z];
                    end

                else
                    disp("no centroids.");
                end
                % appendable_matrix(i, :) = [x, y, nvargs.ball_z-12];
            end 

            for j = 1:n
                unprocessed_output(j, :) = [input_centroids(j, 1), appendable_matrix(j, :)];
            end

            %now, we should neaten up the output. Right now, the function
            %returns very large numbers (~15,000) for inputs of zeros
            %(tbh idrc if it doesn't affect the functionality), 
            index = 1;
            for k = 1:n
                if unprocessed_output(k, 1) ~= 0
                    true_centroids(index, :) = unprocessed_output(k, :); 
                    index = index + 1;
                end
            end 
        end

        function ts_coords = detect_balls(self)
            centroidCoors = self.detect_centroids();
            ts_coords = self.correct_centroids(centroidCoors);
        end       
    
    end
end