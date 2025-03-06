classdef TrajGenerator < handle  
    methods(Static)
        % TODO: Fill in the arguments for these methods
        function coeffs = cubic_traj(q0, qd0, qf, qdf, t0,tf)
        %CUBIC_TRAJ Generates coefficients for a cubic trajectory
        % Inputs:
        %   TODO: Describe input args
        % Outputs:
        %   coeffs: a [1x4] matrix of cubic trajectory coefficients

            % YOUR CODE HERE
            % Hint: to solve the linear system of equations b = Ax,
            %       use x = A \ b
        A = [1, t0, t0^2, t0^3;
                 0, 1,  2 * t0, 3 * t0^2;
                 1, tf, tf^2, tf^3;
                 0, 1, 2 * tf, 3 * tf^2;];

        b = [q0;qd0;qf;qdf];
       
        coeffs = (A\b)';
        end

        % TODO: Fill in the arguments for these methods
        function coeffs = quinitic_traj(q0, qd0, qdd0, qf, qdf, qddf, t0,tf)
        %CUBIC_TRAJ Generates coefficients for a quintic trajectory
        % Inputs:
        %   TODO: Describe input args
        % Outputs:
        %   coeffs: a [1x6] matrix of cubic trajectory coefficients
        A = [1, t0, t0^2, t0^3, t0^4, t0^5;
                0, 1, 2*t0, 3*t0^2, 4*t0^3, 5*t0^4;
                0, 0, 2, 6*t0, 12*t0^2, 20*t0^3;
                1, tf, tf^2, tf^3, tf^4, tf^5;
                0, 1, 2*tf, 3*tf^2, 4*tf^3, 5*tf^4;
                0, 0, 2, 6*tf, 12*tf^2, 20*tf^3];

        b = [q0; qd0; qdd0; qf; qdf; qddf];
            
            
            x = A \ b;
            
          
            coeffs = x.';

        end

        function state = eval_traj(coeff_mat, t) 
        %EVAL_TRAJ Evaluates multiple trajectories
        % Inputs:
        %   coeff_mat: a [nx4] or [nx6] matrix where each row is a set of
        %              cubic or quintic trajectory coefficients
        %   t: a time in seconds at which to evaluate the trajectories
        % Outputs:
        %   state: a [nx1] column vector containing the results of 
        %          evaluating the input trajectories at time t
        n = size(coeff_mat, 2); 
        if n == 4  
                T = [1, t, t^2, t^3]';
            elseif n == 6  
                T = [1, t, t^2, t^3, t^4, t^5]';
            else
                error('Invalid coefficient matrix: must have 4 (cubic) or 6 (quintic) columns.');
        end
         state = coeff_mat * T;  
        end

    function velocity = eval_traj_velocity(coeff_mat, t) 
    %EVAL_TRAJ_VELOCITY Evaluates the velocity of multiple trajectories
    % Inputs:
    %   coeff_mat: a [nx4] or [nx6] matrix where each row is a set of
    %              cubic or quintic trajectory coefficients
    %   t: a time in seconds at which to evaluate the velocity
    % Outputs:
    %   velocity: a [nx1] column vector containing the velocities at time t

    n = size(coeff_mat, 2); 
    if n == 4  
        T_vel = [0, 1, 2*t, 3*t^2]';  % Derivative of cubic polynomial
    elseif n == 6  
        T_vel = [0, 1, 2*t, 3*t^2, 4*t^3, 5*t^4]';  % Derivative of quintic polynomial
    else
        error('Invalid coefficient matrix: must have 4 (cubic) or 6 (quintic) columns.');
    end
    
    velocity = coeff_mat * T_vel;  
end

        
        
           
    end
end
