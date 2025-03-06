% Move robot to a point where Y=0, X > 150, and Z > 150
robot = Robot();
travel_time = 2;
threshold = 20;

% Define start and end positions
pos1 = [0, -100, 350, -90];
pos2 = [0, 100, 350, -90];

% Generate trajectories
x_traj = TrajGenerator.cubic_traj(pos1(1), 0, pos2(1), 0, 0, travel_time);
y_traj = TrajGenerator.cubic_traj(pos1(2), 0, pos2(2), 0, 0, travel_time);
z_traj = TrajGenerator.cubic_traj(pos1(3), 0, pos2(3), 0, 0, travel_time);
alpha_traj = TrajGenerator.cubic_traj(pos1(4), 0, pos2(4), 0, 0, travel_time);

% Store trajectories in coefficient matrix
traj_coeffs = [x_traj; y_traj; z_traj; alpha_traj];

% Initialize arrays for determinant and Y values
determinant_array = zeros(63, 1);
y_array = zeros(63, 1);

% Move robot to start position
robot.interpolate_jp(robot.ik_3001(pos1), 2);
pause(2);

% Execute trajectory
counter = 1;
tic;
while toc < travel_time %% && ~(robot.atSingularity(current_pos(1, :)))   %% take out the && stuff to DEactivate E STOPPINGNGNGNNG
    current_pos = robot.measure_js(true, false);
    if ~robot.atSingularity(current_pos(1, :))
        jacobian = robot.jacob3001(current_pos(1, :));
        determinant = det(jacobian(1:3, 1:3));
        
        ee_traj = TrajGenerator.eval_traj(traj_coeffs, toc);
        robot.interpolate_jp(robot.ik_3001(ee_traj), 0.5);
        
        determinant_array(counter) = determinant;
        y_array(counter) = ee_traj(2);
        counter = counter + 1;
    end
end

% Plot results
figure;
plot(y_array, determinant_array);
xlabel('Y Position');
ylabel('Determinant of Jacobian');
title('Jacobian Determinant vs Y Position');



