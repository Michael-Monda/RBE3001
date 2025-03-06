%% plotting x, y, and z velocities.
armstrong = Robot();
armstrong.writeMotorState(true);

travel_time = 2.7;  % CHOOSE SOMETHING REASONABLE

% Define three [x, y, z, alpha] positions for your end effector that all
% lie upon the x-z plane of your armstrong
end_effector = [150, 30, 223.24, 40;
                332.04, 0, 87.01, 40;
                200.05, -13, 40.81, 15];
%establish x, y, z coordinate series for theoretical
series_x = ([end_effector(:, 1); end_effector(1, 1)])'; %ro-ta-te
series_y = ([end_effector(:, 2); end_effector(1, 2)])';
series_z = ([end_effector(:, 3); end_effector(1, 3)])';

t0=0;
tf=travel_time;
v0=0;
vf=0;
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

actual = zeros(129, 7);
%q = zeros(129, 5);
theo = zeros(129, 4);

for i=1:3
    %calculate the theoretical in here.
    trajmat_x = TrajGenerator.cubic_traj(series_x(1, i), 0, series_x(1, i+1), 0, 0, travel_time);
    trajmat_y = TrajGenerator.cubic_traj(series_y(1, i), 0, series_y(1, i+1), 0, 0, travel_time);
    trajmat_z = TrajGenerator.cubic_traj(series_z(1, i), 0, series_z(1, i+1), 0, 0, travel_time);

    tic;
    
    while toc < travel_time
        index = index+1;
        position = armstrong.measure_js(1,0);
        X = TrajGenerator.eval_traj(c_x(i, :), toc);
        Y = TrajGenerator.eval_traj(c_y(i, :), toc);
        Z = TrajGenerator.eval_traj(c_z(i, :), toc);
        Alpha = TrajGenerator.eval_traj(c_a(i, :), toc);
        jointValue = armstrong.ik_3001([X,Y,Z,Alpha]);
        armstrong.interpolate_jp(jointValue,0.5);
        jacobianVelocity = armstrong.getVelocity();
        actual(index, :) = [toc + travel_time*(i-1), jacobianVelocity'];
        % need to evaluate the generated theoretical in here with
        % eval_velocity
        x = TrajGenerator.eval_traj_velocity(trajmat_x, toc);
        y = TrajGenerator.eval_traj_velocity(trajmat_y, toc);
        z = TrajGenerator.eval_traj_velocity(trajmat_z, toc);
        theo(index, :) = [toc + travel_time*(i-1), x, y, z];
    end
end

%create four arrays of time and theoretical position
time_axis = theo(:, 1)';
theo_x = theo(:, 2)';
theo_y = theo(:, 3)';
theo_z = theo(:, 4)';

figure('Name','Plot of Theoretical Velocities vs Actual');
plot(time_axis, theo_x, 'r-', 'LineWidth', 2); hold on;
plot(time_axis, theo_y, 'Color', [0, 0.7, 0], 'LineWidth', 2);
plot(time_axis, theo_z, 'b-', 'LineWidth', 2);
plot(time_axis, actual(:, 2), 'm-', 'LineWidth',2);
plot(time_axis, actual(:, 3), 'g-', 'LineWidth',2);
plot(time_axis, actual(:, 4), 'c-', 'LineWidth',2);
hold off;

legend({'Theoretical X', 'Theoretical Y', 'Theoretical Z', 'Actual X', 'Actual Y', 'Actual Z'}, 'Location', 'best');
xlabel('Time(s)');
xlim([0, 10]);
ylabel('Velocity');
ylim([-150, 150]);
title('Plot of Theoretical Velocities vs Actual');
grid on;