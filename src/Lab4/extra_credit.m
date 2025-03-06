i%% Numeric IK using the Jacobian pseudoinverse
% Create a Robot instance
bmo = Robot();
% Convergence criteria and maximum iterations
convergence = 1e-3;
max_iterations = 1000;

% Initial guess for joint angles (in degrees)
thetaSet = [0, 0, 0, 0];
% Desired end-effector position (x, y, z)
desired_position = [318.65, 97.42, 106.8];
for iteration = 1:max_iterations
    % Compute forward kinematics at the current joint angles.
    % (Assume fk_3001 returns a 4x4 homogeneous transformation matrix.)
    curr_T = bmo.fk_3001(thetaSet);
    curr_pos = curr_T(1:3, 4);  % or use (1:3,3) if that's how your function works
        %disp(curr_pos);
    % Calculate the error (as a column vector)
    error = desired_position' - curr_pos;
    % Check for convergence
    if norm(error) < convergence
        disp('Convergence achieved.');
        break;
    end
    J = bmo.jacob3001(thetaSet);
    disp(J)
    % Extract the positional part of the Jacobian (first 3 rows)
    J_pos = J(1:3, :);
    % Compute the Moore-Penrose pseudoinverse of the positional Jacobian
    J_pinv = pinv(J_pos);
    % Compute the incremental change in joint angles
    delta_theta = J_pinv * error;
    % Update joint angles (note: add the delta because error = desired - current)
    thetaSet = thetaSet + delta_theta';
    % (Optional) Display the error norm for debugging/monitoring convergence
    fprintf('Iteration %d: error norm = %e\n', iteration, norm(error));
end
disp('Derived Joint Angles (degrees):');
disp(thetaSet);
