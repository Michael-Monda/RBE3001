function compare_cubic_quintic_complex()
    % COMPARE_CUBIC_QUINTIC_COMPLEX
    %   Generates two 1-D trajectories (cubic & quintic) with the SAME
    %   start/end times, positions, and velocities, but non-zero velocities
    %   for a more complex scenario than the trivial [0,0].

    %% 1) Define boundary conditions
    % Let's pick some non-zero initial/final velocity to increase complexity:
    q0   = 299.5;   % initial position
    qd0  = 0;   % initial velocity (non-zero)
    qf   = 306.96;  % final position
    qdf  = 0;  % final velocity (non-zero)
    t0   = 0;   % total duration (seconds)
    tf =2.6;
    % For the quintic, we can set initial & final acceleration to zero
    % (or any other values), but the instructions only REQUIRE time/pos/vel match.
    qdd0 = 0;   
    qddf = 0;
    interpolate_time = 2.6;


x_seq = [332.04, 299.50, 306.96, 332.04];
y_seq = [0, 0, 0, 0];
z_seq = [87.41, 108.81, 223.24, 87.41];

for i = 1:3
    % Generate cubic trajectory coefficients for x, y, z
    xtraj = TrajGenerator.cubic_traj(x_seq(i), 0, x_seq(i+1), 0, (i-1)*interpolate_time, i*interpolate_time);
    ytraj = TrajGenerator.cubic_traj(y_seq(i), 0, y_seq(i+1), 0, (i-1)*interpolate_time, i*interpolate_time);
    ztraj = TrajGenerator.cubic_traj(z_seq(i), 0, z_seq(i+1), 0, (i-1)*interpolate_time, i*interpolate_time);
    disp('x:');
    disp(xtraj);
    disp('y:');
    disp(ytraj);
    disp('z:');
    disp(ztraj);
    disp()
end

    coeffs_cubic   = TrajGenerator.cubic_traj(q0, qd0, qf, qdf, t0,tf);
    coeffs_quintic = TrajGenerator.quinitic_traj(q0, qd0, qdd0, ...
                                                qf, qdf, qddf, t0,tf);
    de_cubic = diff(coeffs_cubic);

    


    disp('Quintic coefficients:');
    disp(coeffs_quintic);

    %% 3) Evaluate over a time vector
    tvec = linspace(0, tf, 100);  % 100 sample points
    N = length(tvec);

    pos_cubic   = zeros(1, N);
    pos_quintic = zeros(1, N);

    vel_c =zeros(1, N);
    vel_q =zeros(1, N);
   

    for i = 1:N
        % Evaluate the cubic at tvec(i)
        state_c = TrajGenerator.eval_traj(coeffs_cubic, tvec(i));   % [pos, vel, acc]
        velocity_c = TrajGenerator.eval_traj_velocity(coeffs_cubic,tvec(i));
        % Evaluate the quintic at tvec(i)
        state_q = TrajGenerator.eval_traj(coeffs_quintic, tvec(i)); % [pos, vel, acc]
        velocity_q = TrajGenerator.eval_traj_velocity(coeffs_quintic,tvec(i));
        
        pos_cubic(i)   = velocity_c;  % position is the first element
        pos_quintic(i) = velocity_q;
        vel_c(i) = state_c;
        vel_q(i) = state_q;
    end

    %% 4) Plot both on the same figure
    figure('Name','Cubic vs Quintic (Complex)');
    plot(tvec, pos_cubic, 'r-', 'LineWidth', 2); hold on;
    plot(tvec, pos_quintic, 'b--', 'LineWidth', 2);

    legend({'Cubic','Quintic'}, 'Location','Best');
    xlabel('Time (s)');
    ylabel('Position');
    title('Comparison: Cubic vs Quintic with Non-zero Velocities');
    grid on;

    figure('Name','Cubic vs Quintic (Complex)');
    plot(tvec, vel_c, 'r-', 'LineWidth', 2); hold on;
    plot(tvec, vel_q, 'b--', 'LineWidth', 2);

    legend({'Cubic','Quintic'}, 'Location','Best');
    xlabel('Time (s)');
    ylabel('Position');
    title('Comparison: Cubic vs Quintic with Non-zero Velocities');


    %% 5) Demonstrate eval_traj with an SA-provided test case
    % Example: Suppose the SA says we should test these cubic coefficients at t=1.0
    test_coeffs = [0, 0, 7.5, -2.5];  % e.g., known from a simpler scenario
    t_test      = 1.0;
    state_test  = TrajGenerator.eval_traj(test_coeffs, t_test);
    disp('SA test case result at t=1.0 (coeffs = [0,0,7.5,-2.5]):');
    disp(state_test);  % e.g., might be [5, 7.5, 0] if that is the known correct result
end
