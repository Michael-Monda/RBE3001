%% USE THIS FILE TO GET SIGN OFF 3

robot = Robot();  % Create a robot object to use
interpolate_time = 3; % TODO: choose a reasonable time to interpolate

%% Send the robot to its 0 position
% We don't need to collect data yet, as we're just putting the robot at its
% starting point
p = [0, -85, 88, 0];
robot.servo_jp(p);
pause(1);

% YOUR CODE HERE

%% Send the robot to an arbitrary position
yaw = -45;
pos1 = [yaw 13 -23 16];  % establish the position.

point = zeros(1, 4);
dataArr = zeros(200, 5);

robot.writeJoints(pos1); % move to the resting position.
index = 0;

tic; % Start timer
while toc < interpolate_time
    disp(robot.measure_js(1, 0)); % Read joint values
    point = robot.measure_js(1, 0);
    index = index + 1;
    dataArr(index, :) = [toc, point];
end
    


% Initialize another array for the timestamps (or use the same array as the
% positions, your call)
%% We (Jace, Michael, Leo) created one matrix to store timestamps, and the corresponding positional data.

%% Make your figure
% To use subfigures, you'll use the subplots feature of MATLAB.
% https://www.mathworks.com/help/matlab/ref/subplot.html

% In each subplot you create, you can use 'plot' to plot one joint value vs
% time
% https://www.mathworks.com/help/matlab/ref/plot.html

% Remember titles, labels, and units!
% extract time intervals
time = dataArr(:, 1);

% and motor positions
motor0pos = dataArr(:, 2);
motor1pos = dataArr(:, 3);
motor2pos = dataArr(:, 4);
motor3pos = dataArr(:, 5);

figure;
% motor0
subplot(4, 1, 1)
plot(time, motor0pos, 'o');
xlabel('Time (s)');
ylabel('Position (degrees)');
title('Base Joint Position vs Time')
grid on;

% motor1
subplot(4, 1, 2);
plot(time, motor1pos, 'o');
xlabel('Time (s)');
ylabel('Position (degrees)');
title('First Joint Position vs Time');
grid on;

% motor2
subplot(4, 1, 3);
plot(time, motor2pos, 'o');
xlabel('Time (s)');
ylabel('Position (degrees)');
title('Second Joint Position vs Time');
grid on;

% motor3
subplot(4, 1, 4);
plot(time, motor3pos, 'o');
xlabel('Time (s)');
ylabel('Position (degrees)');
title('Third Joint Position vs Time');
grid on;

%% Calculate time step statistics
% MATLAB may have some functions for the requested statistics...