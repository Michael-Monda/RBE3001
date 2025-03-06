clear; clc;

robot = Robot();

% --- Simulation Parameters ---
threshold = 1e-3;   % Singularity detection threshold
x_fixed   = 0;      % Fixed x position
z_fixed   = 0;      % Fixed z position
y_start   = 0;      % Starting y position
dy        = 0.01;   % Increment in y for each step
maxSteps  = 1000;   % Maximum number of steps to prevent an infinite loop

% Arrays to store y positions and determinant values
y_positions = [];
determinants = [];

fprintf('Starting trajectory along the y-axis...\n');

% --- Trajectory Simulation ---
y_current = y_start;
for step = 1:maxSteps
    target_position = [x_fixed, y_current, z_fixed];
    robot.moveTo(target_position);
    pause(0.05);
    
    % Compute the Jacobian and its determinant (of J*J')
    J = robot.computePositionJacobian();
    JJt = J * J';
    detJJt = det(JJt);
    
    y_positions(end+1) = y_current;     
    determinants(end+1) = detJJt;
    
    if detJJt < threshold
        fprintf('Singularity detected at y = %.4f\n', y_current);
        break;
    end
    
    y_current = y_current + dy;
end

if step == maxSteps
    fprintf('Trajectory completed without reaching a singularity.\n');
end

figure;
plot(y_positions, determinants, '-o', 'LineWidth', 1.5);
xlabel('Y Position');
ylabel('det(JJ^T)');
title('Determinant of the Jacobian vs Y Position');
grid on;

classdef Robot
    properties
        % The robot stores its current end-effector position.
        current_position = [0, 0, 0];
    end
    
    methods
        function moveTo(obj, target_position)
            % Simulate moving to the target position by simply storing it.
            obj.current_position = target_position;
        end
        
        function J = computePositionJacobian(obj)
            
            y = obj.current_position(2);
            f = 1 - y;
            J = [1,  0, 0;
                 0,  f, 0;
                 0,  0, 1];
        end
    end
end
