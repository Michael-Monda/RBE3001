% Create a robot object
robot = Robot();
travel_time = 2.7;  % CHOOSE SOMETHING REASONABLE

% Define three [x, y, z, alpha] positions for your end effector that all
% lie upon the x-z plane of your robot
end_effector = [150, 0, 223.24, 40;
                332.04, 0, 87.01, 40;
                200.05, 0, 40.81, 15];

% For each leg of your triangle, calculate the TASK SPACE trajectory
% between each vetex. Remember, to calculate a task space trajectory 
% you will need to create individual trajectories for x, y, z, and alpha.

% Move your robot to the first vertex
%robot.servo_jp(robot.ik_3001());
pause(0.5);
t0=0;
tf=travel_time;
v0=0;
vf=0;
% Create a for loop that sends your robot to each vertex, recording the
% JOINT-SPACE position the entire time
index = 0;
c_x = zeros(3, 4);
c_y = zeros(3, 4);
c_z = zeros(3, 4);
c_a = zeros(1, 4);
for i = 1:3
    q0 = end_effector(i,:);
    qf = end_effector(mod(i,3)+1,:);
    
    c_x(i,:) = TrajGenerator.cubic_traj(q0(1),v0,qf(1),vf,t0,tf);
    c_y (i,:) = TrajGenerator.cubic_traj(q0(2),v0,qf(2),vf,t0,tf);
    c_z (i,:) = TrajGenerator.cubic_traj(q0(3),v0,qf(3),vf,t0,tf);
    c_a (i,:) = TrajGenerator.cubic_traj(q0(4),v0,qf(4),vf,t0,tf);
end

p = zeros(700, 7);
q = zeros(700, 5);
v = zeros(700, 5);

for i=1:3
    tic;
    
    while toc < travel_time
        % Evaluate your trajectory using eval_traj
        index = index+1;
        position = robot.measure_js(1,0);
        X = TrajGenerator.eval_traj(c_x(i, :), toc);
        Y = TrajGenerator.eval_traj(c_y(i, :), toc);
        Z = TrajGenerator.eval_traj(c_z(i, :), toc);
        Alpha = TrajGenerator.eval_traj(c_a(i, :), toc);
        jointValue = robot.ik_3001([X,Y,Z,Alpha]);
        robot.interpolate_jp(jointValue,0.5);
        jacobianVelocity = robot.getVelocity();
        p(index, :) = [toc, jacobianVelocity'];
    end
end