
% Ultimately it's up to you to decide how to organize this, but the
% "detect_balls" and "pick_up_ball" functions should do a lot of the work
% for you.

armstrong = Robot();
%imager = ImageProcessor();
home = [0, 0, 0, 0];
open = -35;
close = 55;

%define deposit locs in terms of joint angles. Need to redo bc last time was too low.
redbin = [-70.5781   37.8809  -45.0879  60.0508];
orangebin = [-46.9336   38.5840  -38.6719   61.8750];
yellowbin = [46.9336   38.5840  -38.6719   61.8750]; %
greenbin = [70.5781   37.8809  -45.0879  60.0508]; % [70.5781   37.8809  -45.0879  60.0508];
bluebin = [23.1152   48.7793  -29.0918   75.3223];
alpha = 90;
k = 20;
target_matrix = zeros(k, 4);
above = 35; %mm above tgt

disp("Please place a ball, you have 10 sec");
pause(1);disp("9");pause(1);disp("8");pause(1);disp("7");pause(1);disp("6");pause(1);disp("5");pause(1);
disp("4");pause(1);disp("3");pause(1);disp("2");pause(1);disp("1");disp("Lets Go!");

armstrong.blocking_js_move(home);
%capture centroids
target_matrix(:, :) = imager.detect_balls();
%open the servo
armstrong.gripper.writePosition(open);

%% sorting algorithm
while true
    armstrong.blocking_js_move(home); %reset position between detects.
    pause(1);
    target_matrix(:, :) = imager.detect_balls();
    %unless no color is detected...
    if target_matrix(1, 1) ~= 0
        armstrong.blocking_ts_move([target_matrix(1, 2:3), above, alpha]);
        armstrong.blocking_ts_move([target_matrix(1, 2:4), alpha]);
        armstrong.gripper.writePosition(close);
        pause(1);
        armstrong.blocking_ts_move([target_matrix(1, 2:3), above+15, alpha]);
        pause(1);
        armstrong.blocking_js_move(home);
        if target_matrix(1, 1) == 1         %red
            disp("red detected");
            armstrong.blocking_js_move(redbin);
            armstrong.gripper.writePosition(open);
            disp("red deposited");
        elseif target_matrix(1, 1) == 2     %orange
            disp("orange detected");
            armstrong.blocking_js_move(orangebin);
            armstrong.gripper.writePosition(open);
            disp("orange deposited");
        elseif target_matrix(1, 1) == 3     %yellow
            disp("yellow detected");
            armstrong.blocking_js_move(yellowbin);
            armstrong.gripper.writePosition(open);
            disp("yellow deposited");
        elseif target_matrix(1, 1) == 4     %green
            disp("green detected");
            armstrong.blocking_js_move(greenbin);
            armstrong.gripper.writePosition(open);
            disp("green deposited");
        elseif target_matrix(1, 1) == 5     %blue
            disp("blue detected");
            armstrong.blocking_js_move(bluebin);
            armstrong.gripper.writePosition(open);
            disp("blue deposited");
        else
            disp("sensing error. please reset.");
        end
        
    end
end