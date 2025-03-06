% Create a robot object
armstrong = Robot();
armstrong.writeMotorState(true);

% Define three joint positions to form the vertices of your triangle
% Make sure theta_1 = 0 for all of these configurations!

% define 3 positions in terms of theta2, 3, and 4
initial = [0, 13, -23, 16];
vertex1 = [13, -23, 16];
vertex2 = [37, -23, 16];
vertex3 = [23, 17, -33];
trajectory = [vertex2; vertex3; vertex1];

index = 0;
triangleArr = zeros(700, 5);
interpolate_time = 2.6;
armstrong.writeTime(interpolate_time);


% Move your robot to the first vertex
% disp('start');
armstrong.writeJoints([0, vertex1]);
pause(interpolate_time);
% disp('initialized');


% Create a for loop that sends your robot to each vertex, recording the
% joint-space position the entire time
for i=1:3
    disp(trajectory(i, :));
    armstrong.writeJoints([0, trajectory(i, :)]);

    tic; % Start timer
    while toc < interpolate_time
        % disp(robot.measure_js(1, 0)); % Read joint values
        point = armstrong.measure_js(1, 0);
        index = index + 1;
        triangleArr(index, :) = [toc, point];
    end
end

% Loop through all collected data and convert it from joint-space to task
% space using your fk3001 function
timex = triangleArr(:, 1);

% this array will intake the joint-space data (degrees) and pass to fk_3001
% for conversion to DHTables
convertable = triangleArr(:, 3:5);

% iterate through convertable according to its size and convert each row
% into task-space form
iterations = size(triangleArr);
newMatrix = zeros(iterations(1,1), 3);
for i = 1:iterations
    ht = armstrong.fk_3001([0, convertable(i, :)]); 
    newMatrix(i, :) = ht(1:3, 4)';
end
% extract the data from the converted matrix and write into positions of X,
% Y, and Z.
xpos = newMatrix(:, 1);
ypos = newMatrix(:, 2);
zpos = newMatrix(:, 3);
theta2 = triangleArr(:, 3);
theta3 = triangleArr(:, 4);
theta4 = triangleArr(:, 5);

% Plot the trajectory of your robot in x-y-z space using scatter3
% https://www.mathworks.com/help/matlab/ref/scatter3.html
figure;
scatter3(xpos, ypos, zpos, 50, timex, 'filled'); 
xlabel('End-Effector X Position');
ylabel('End-Effector Y Position');
zlabel('End-Effector Z Position');
title('Task-Space Trajectory');
grid on;

% Plot the trajectory of your robot in theta2-theta3-theta-4 space using
% scatter3
figure;
scatter3(theta2, theta3, theta4, 50, timex, 'filled'); 
xlabel('Joint 1 Position (degrees)');
ylabel('Joint 2 Position (degrees)');
zlabel('Joint 3 Position (degrees)');
title('Joint-Space Trajectory');
grid on;