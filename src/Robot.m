% (c) 2023 Robotics Engineering Department, WPI
% Skeleton Robot class for OpenManipulator-X Robot for RBE 3001

classdef Robot < OM_X_arm
    % Many properties are abstracted into OM_X_arm and DX_XM430_W350. classes
    % Hopefully, you should only need what's in this class to accomplish everything.
    % But feel free to poke around!
    properties
        mDim; % Stores the robot link dimentions (mm)
        mOtherDim; % Stores extraneous second link dimensions (mm)
        mSingularity;
        limitH;
        limitL;
    end
    methods
        % Creates constants and connects via serial
        % Super class constructor called implicitly
        % Add startup functionality here
        function self = Robot()
            % Change robot to position mode with torque enabled by default
            % Feel free to change this as desired
            self.writeMode('p');
            self.writeMotorState(true);
            self.mDim = [36.076, 60.25, 130.23, 124, 133.4]; % (mm)
            self.mOtherDim = [128, 24]; % (mm)
            self.mSingularity = 500000; %1e6;   %thresh for singularity detection.
            self.limitH = 100;
            self.limitL = -100;

            % Set the robot to move between positions with a 5 second profile
            % change here or call writeTime in scripts to change
            self.writeTime(2);
        end

        % Sends the joints to the desired angles
        % goals [1x4 double] - angles (degrees) for each of the joints to go to
        function writeJoints(self, goals)
            if true%checkSafe(goals)
                goals = mod(round(goals .* DX_XM430_W350.TICKS_PER_DEG + DX_XM430_W350.TICK_POS_OFFSET), DX_XM430_W350.TICKS_PER_ROT);
                self.bulkReadWrite(DX_XM430_W350.POS_LEN, DX_XM430_W350.GOAL_POSITION, goals);
            end
        end

        % Creates a time based profile (trapezoidal) based on the desired times
        % This will cause writePosition to take the desired number of
        % seconds to reach the setpoint. Set time to 0 to disable this profile (be careful).
        % time [double] - total profile time in s. If 0, the profile will be disabled (be extra careful).
        % acc_time [double] - the total acceleration time (for ramp up and ramp down individually, not combined)
        % acc_time is an optional parameter. It defaults to time/3.
      
        function writeTime(self, time, acc_time)
            if (~exist("acc_time", "var"))
                acc_time = time / 3;
            end

            time_ms = time * DX_XM430_W350.MS_PER_S;
            acc_time_ms = acc_time * DX_XM430_W350.MS_PER_S;

            disp("time")
            disp(time_ms)
            disp("acc time")
            disp(acc_time_ms)

            self.bulkReadWrite(DX_XM430_W350.PROF_ACC_LEN, DX_XM430_W350.PROF_ACC, acc_time_ms);
            self.bulkReadWrite(DX_XM430_W350.PROF_VEL_LEN, DX_XM430_W350.PROF_VEL, time_ms);
        end
        
        % Sets the gripper to be open or closed
        % Feel free to change values for open and closed positions as desired (they are in degrees)
        % open [boolean] - true to set the gripper to open, false to close
        function writeGripper(self, open)
            if open
                self.gripper.writePosition(-35);
            else
                self.gripper.writePosition(55);
            end
        end

        % Sets position holding for the joints on or off
        % enable [boolean] - true to enable torque to hold last set position for all joints, false to disable
        function writeMotorState(self, enable)
            self.bulkReadWrite(DX_XM430_W350.TORQUE_ENABLE_LEN, DX_XM430_W350.TORQUE_ENABLE, enable);
        end

        % Supplies the joints with the desired currents
        % currents [1x4 double] - currents (mA) for each of the joints to be supplied
        function writeCurrents(self, currents)
            currentInTicks = round(currents .* DX_XM430_W350.TICKS_PER_mA);
            self.bulkReadWrite(DX_XM430_W350.CURR_LEN, DX_XM430_W350.GOAL_CURRENT, currentInTicks);
        end

        % Change the operating mode for all joints:
        % https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/#operating-mode11
        % mode [string] - new operating mode for all joints
        % "current": Current Control Mode (writeCurrent)
        % "velocity": Velocity Control Mode (writeVelocity)
        % "position": Position Control Mode (writePosition)
        % Other provided but not relevant/useful modes:
        % "ext position": Extended Position Control Mode[
        % "curr position": Current-based Position Control Mode
        % "pwm voltage": PWM Control Mode
        function writeMode(self, mode)
            switch mode
                case {'current', 'c'} 
                    writeMode = DX_XM430_W350.CURR_CNTR_MD;
                case {'velocity', 'v'}
                    writeMode = DX_XM430_W350.VEL_CNTR_MD;
                case {'position', 'p'}
                    writeMode = DX_XM430_W350.POS_CNTR_MD;
                case {'ext position', 'ep'} % Not useful normally
                    writeMode = DX_XM430_W350.EXT_POS_CNTR_MD;
                case {'curr position', 'cp'} % Not useful normally
                    writeMode = DX_XM430_W350.CURR_POS_CNTR_MD;
                case {'pwm voltage', 'pwm'} % Not useful normally
                    writeMode = DX_XM430_W350.PWM_CNTR_MD;
                otherwise
                    error("setOperatingMode input cannot be '%s'. See implementation in DX_XM430_W350. class.", mode)
            end

            lastVelTimes = self.bulkReadWrite(DX_XM430_W350.PROF_VEL_LEN, DX_XM430_W350.PROF_VEL);
            lastAccTimes = self.bulkReadWrite(DX_XM430_W350.PROF_ACC_LEN, DX_XM430_W350.PROF_ACC);

            self.writeMotorState(false);
            self.bulkReadWrite(DX_XM430_W350.OPR_MODE_LEN, DX_XM430_W350.OPR_MODE, writeMode);
            self.writeTime(lastVelTimes(1) / 1000, lastAccTimes(1) / 1000);
            self.writeMotorState(true);
        end

        % Gets the current joint positions, velocities, and currents
        % readings [3x4 double] - The joints' positions, velocities,
        % and efforts (deg, deg/s, mA)
        function readings = getJointsReadings(self)
            readings = zeros(3,4);
            
            readings(1, :) = (self.bulkReadWrite(DX_XM430_W350.POS_LEN, DX_XM430_W350.CURR_POSITION) - DX_XM430_W350.TICK_POS_OFFSET) ./ DX_XM430_W350.TICKS_PER_DEG;
            readings(2, :) = self.bulkReadWrite(DX_XM430_W350.VEL_LEN, DX_XM430_W350.CURR_VELOCITY) ./ DX_XM430_W350.TICKS_PER_ANGVEL;
            readings(3, :) = self.bulkReadWrite(DX_XM430_W350.CURR_LEN, DX_XM430_W350.CURR_CURRENT) ./ DX_XM430_W350.TICKS_PER_mA;
        end

        % Sends the joints at the desired velocites
        % vels [1x4 double] - angular velocites (deg/s) for each of the joints to go at
        function writeVelocities(self, vels)
            vels = round(vels .* DX_XM430_W350.TICKS_PER_ANGVEL);
            self.bulkReadWrite(DX_XM430_W350.VEL_LEN, DX_XM430_W350.GOAL_VELOCITY, vels);
        end
    
        %% LAB 1-----------------------------------------------------------
        % ALL code for lab 1 goes in this section ONLY

        function servo_jp(self, q)
        %SERVO_JP Send robot to a joint configuration  
            if (q(1) < self.limitL) || (q(1) > self.limitH) 
                disp("Theta1 exceeds permitted position!")
                return;
            end
            self.writeJoints(q);
            pause(3);

        end
        
        function interpolate_jp(self, q, time)
        %INTERPOLATE_JP Send robot to a joint configuration over a period of time
            if (q(1) < self.limitL) || (q(1) > self.limitH) 
                disp("Theta1 exceeds permitted position!")
                return;
            end
            self.writeTime(time);    %may have to do movement -> time instead of this
            self.writeJoints(q);

        end

        function q_curr = measure_js(robot, GETPOS, GETVEL)        
            % This line gets the current positions of the joints
            % (robot.bulk[ReadWrite(DX_XM430_W350.POS_LEN, DX_XM430_W350.CURR_POSITION) - DX_XM430_W350.TICK_POS_OFFSET) ./ DX_XM430_W350.TICKS_PER_DEG;
            
            % This line gets the current joint velocities
            % (robot.bulkReadWrite(DX_XM430_W350.VEL_LEN, DX_XM430_W350.CURR_VELOCITY) ./ DX_XM430_W350.TICKS_PER_ANGVEL);
            
            % YOUR CODE HERE
            % Hint: Only collect the data you need; serial read operations
            %       are expensive!

            q_curr = zeros(1,4);
          
            if GETPOS
                q_curr(1, :) = (robot.bulkReadWrite(DX_XM430_W350.POS_LEN, DX_XM430_W350.CURR_POSITION) - DX_XM430_W350.TICK_POS_OFFSET) ./ DX_XM430_W350.TICKS_PER_DEG;
            end
            if GETVEL
                q_curr(1, :) = robot.bulkReadWrite(DX_XM430_W350.VEL_LEN, DX_XM430_W350.CURR_VELOCITY) ./ DX_XM430_W350.TICKS_PER_ANGVEL;
            end
        end
        %% END LAB 1 CODE -------------------------------------------------
        %% BEGIN LAB 2 CODE -----------------------------------------------
        function ht = dh2mat(self, dh_row)
        %DH2MAT Gives the transformation matrix for a given row of a DH table
        %% TODO: Review the following line to check 'self' implementation.
            % dh_row = self.DHTable(dh_row, :);

            % extract DH params from dh_row
            thetaZ = dh_row(1);
            d = dh_row(2);
            a = dh_row(3);
            alphaX = dh_row(4);

            % convert from degrees to radians, because MATLAB's trig
            % functions only accept radian angle measurements.
            theta = deg2rad(thetaZ);
            alpha = deg2rad(alphaX);

            % configure the homogenous transformation matrix according to
            % the image in the lab handout
            ht = [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha),  a*cos(theta);
                  sin(theta), cos(theta)*cos(alpha),  -cos(theta)*sin(alpha), a*sin(theta);
                  0,          sin(alpha),             cos(alpha),             d;
                  0,          0,                      0,                      1];
        end

        function ht = dh2fk(self, dh_tab)
        %DH2MAT_R Calculates FK from a DH table
            
            % Initialize the final transformation matrix as identity matrix
            ht = eye(4);
    
            % Number of rows in the DH table
            num_joints = size(dh_tab, 1);
    
            % Loop through each row of the DH table
            % convert from degrees to radians, because MATLAB's trig
            % functions only accept radian angle measurem
            for i = 1:num_joints
                % Get the DH parameters for the current joint
                dh_row = dh_tab(i, :);
            % convert from degrees to radians, because MATLAB's trig
            % functions only accept radian angle measurem
        
            % convert from degrees to radians, because MATLAB's trig
            % functions only accept radian angle measurem
                % Compute the transformation matrix for the current joint
                ht_i = self.dh2mat(dh_row);
        
                % Update the final transformation matrix by post-multiplying
                ht = ht * ht_i;
            end
        end


        % YOUR CODE HERE
        % Write a function fk3001 that takes in an array of four joint
        % values and uses your matlabFunction'd FK to generate the FK of
        % your robot
        function ht = fk_3001(self, thetaArr)
%             theta = deg2rad(theta);
            ht = dhtable(thetaArr(1), thetaArr(2), thetaArr(3), thetaArr(4));
        end 
        % Hint: All non-static methods (which this is) should take "self" as
        % their first argument
        %% END LAB 2 CODE -------------------------------------------------
        %% BEGIN LAB 3 CODE -----------------------------------------------
        %(Mark is having a look at it. Check slack tomorrow.)
        function qs = ik_3001(self, pos)
            %IK3001_R Calculates IK for the OMX arm
            % Inputs:
            %    pos: a [nx4] matrix [x, y, z, alpha] representing a target
            %    pose in task space
            % Outputs: 
            %    qs: a [1x4] matrix representing 
            %    the joint values needed to reach the target position

            L0 = 36.076;
            L1 = self.mDim(1) + self.mDim(2);
            L2 = self.mDim(3);
            L3 = self.mDim(4);
            L4 = self.mDim(5);
            

            %extract from inputs
            x = pos(1);
            y = pos(2);
            z = pos(3);
            alpha = pos(4);

            %now that all the data has been vetted, we can define some
            %constants unique to this arm position.
       
            z4 = z + L4*sind(alpha) - L1;
            r = sqrt((x^2 + y^2)); %we will be working in the "z-r plane."
            r4 = r- L4 * cosd(alpha);

            %hardcode the value of angle psi, the offset which disrupts the
            %linearity of joints 1 and 2:
            psi = atand(24/128);         
            offset1 = 90-psi;
            
            %validate data in accordance with criterion
            if z < 0
                disp('z-value is negative')
                qs = NaN(1, 4);
                return;
            end
            
%             extension_maximum = 387.64;
%             curr_extension = sqrt(x^2 + y^2 + z^2);
%             if curr_extension > extension_maximum
%                 disp('error: this coordinate is unreachable.')
%                 return;
%             end
            %this decouples the end effector from joint 4.
            %% INVERSE KINEMATICS TIME:

            %THETA1
            theta1 = atan2d(y, x);

            % THETA2
            d2_1 = (L2^2 + r4^2 + z4^2 - L3^2);
            d2_2 = (2*L2*sqrt((r4^2 + z4^2)));
            d2 = d2_1/d2_2;
            c2 = sqrt((1 - d2^2));
            beta = atan2d(c2, d2);
            delta = atan2d(r4,z4);
            theta2_1 = delta - psi - beta;
            theta2_2 = delta - psi + beta;

            % THETA3
            d3_1 = (L2^2 +L3^2 - r4^2 -z4^2);
            d3_2 = (2 * L3 * L2);
            d3= d3_1/d3_2;
            c3 = sqrt((1 - d3^2));
            gamma = atan2d(c3, d3);
            theta3_1 = 180 - gamma - offset1;
            theta3_2 = 180 + gamma - offset1;
            
            %check validity of angles.
            if (-89 < theta2_1 && theta2_1 < 89)
                theta2 = theta2_1;
            disp('case 1')
            end

            if (-89 < theta2_2 && theta2_2 < 89)
                theta2 = theta2_2;
            disp('case 2')
            end

            if (-89 < theta2_1 && theta2_1 < 89) && (-89 < theta2_2 && theta2_2 < 89)
                disp('Multiple valid Theta2 answers!');
                if (theta2_1 < theta2_2)
                    theta2 = theta2_1;
                end
                if (theta2_2 < theta2_1)
                    theta2 = theta2_2;
                end
            end
            

            if (-88 < theta3_1 && theta3_1 < 88)
                theta3 = theta3_1;
            disp('case 3')
            end

            if (-88 < theta3_2 && theta3_2 < 88)
                theta3 = theta3_2;
                disp('case 4')
            end

            if (-88 < theta3_1 && theta3_1 < 88) && (-88 < theta3_2 && theta3_2 < 88)
                disp('Multiple valid Theta3 answers!');
                if (theta3_1 < theta3_2)
                    theta3 = theta3_1;
                end
                if (theta3_2 < theta3_1)
                    theta3 = theta3_2;
                end
            end

            %THETA4
            theta4= alpha - theta2 - theta3;
            
            % Hint: MATLAB debugger is your very best friend in the
            % entire world right now

            % TODO: Before running calculations, make sure
            %   1. The length of the x, y, z vector is less than the
            %      length of the extended robot arm
            %   2. The z-value is not negative
            % Feel free to add any other invalid input conditions,
            % but these are the bare minimum

            qs = [theta1, theta2, theta3, theta4];
        
        end
        %% END LAB 3 CODE -------------------------------------------------
        %% BEGIN LAB 4 CODE -----------------------------------------------
        function j = jacob3001(self, qpos) 
            %JACOB3001 Calculates the jacobian of the OMX arm
            % Inputs:
            %   qpos: a [1x4] matrix composed of joint positions
            % Outputs:
            %   j: a [6x4] jacobian matrix of the robot in the given pos

            % extract joint positions from passed qpos
            theta1 = qpos(1);
            theta2 = qpos(2);
            theta3 = qpos(3);
            theta4 = qpos(4);

            j = jacob3001_sym(theta1, theta2, theta3, theta4);
        end

        function vs = dk3001(self, qpos, qvel)
            %DK3001 Calculates the forward velocity kinematics of the OMX
            %arm
            % Inputs:
            %   qpos: a [1x4] matrix composed of joint positions
            %   qvel: a [1x4] matrix composed of joint angular velocities
            % Outputs:
            %   vs: a [6x1] matrix representing the velocity of the end 
            %       effector in the base frame of the robot
            % YOUR CODE HERE

            qvel = qvel * (pi/180); %force to be in rads/s
            jm = self.jacob3001(qpos);
            vs = jm * qvel.';
            %disp(vs);
        end

        function v = getVelocity(self)
            v = self.getJointsReadings();
            pos = v(1, :);
            vel = v(2, :); %if errata in readings, correct here.
            v = self.dk3001(pos, vel);
        end
            
        function isSingular = atSingularity(obj, qpos)
            % atSingularity: Checks if the robot's configuration is near a singularity.
            %
            % Inputs:
            %   threshold - A small positive number that defines the singularity limit.
            %
            % Output:
            %   isSingular - Boolean value. True if the determinant of (J*J') is below the threshold.
            
            % Compute the position Jacobian (assumed to be a 3x4 matrix)
            Jprime = obj.jacob3001(qpos);
            J = Jprime(1:3,1:3);
            
            % Compute the product J*J'
            %JJt = J * J';
            
            % Compute the determinant of JJt
            detJJt = det(J);
            
            % Determine if we are near a singularity (determinant below threshold)
            isSingular = (detJJt < obj.mSingularity);
            disp("it is...");
            disp(detJJt);
        end

        % YOUR CODE HERE: Write a function atSingularity as described in
        % the lab document
        %% END LAB 4 CODE -------------------------------------------------
        %% BEGIN LAB 5 CODE -----------------------------------------------
    function blocking_js_move(self, qpos, nvargs)
        arguments
            self Robot;
            qpos double;
            nvargs.time double = 2;
        end
        %BLOCKING_JS_MOVE moves the robot to a position in joint space
        %before exiting
        % Inputs:
        %   qpos: a [1x4] matrix of joint positions
        %   time: (optional): an integer representing desired travel time
        %         in seconds. Default time: 2 seconds.
        
        % YOUR CODE HERE
        % NOTE: this funciton should not exit until the robot has stopped
        % movin< 
        self.interpolate_jp(qpos,nvargs.time);
        tic
        while toc < nvargs.time
              
        end
    end

    function blocking_ts_move(self, pos, nvargs)
        arguments
            self Robot;
            pos double;
            nvargs.time double = 2;
            nvargs.mode string = "cubic"
        end
        %BLOCKING_TS_MOVE moves the robot in a straight line in task space 
        %to the target position before exiting
        % Inputs:
        %   pos: a [1x4] matrix representing a target x, y, z, alpha
        %   time (optional): an integer representing desired travel time
        %         in seconds. Default time: 2 seconds.
        %   mode (optional): a string "cubic" or "quintic" indicating what 
        %                    type of trajectory to utilize

        % YOUR CODE HERE
        js = self.ik_3001(pos);
        self.interpolate_jp(js,nvargs.time);
        tic
        while toc < nvargs.time

        end
        % NOTE: this funciton should not exit until the robot has stopped
        % moving
    end

    % @@@@@@@@@@@@@@
    % YOUR CODE HERE
    % @@@@@@@@@@@@@@
    % Write a function pick_up_ball that satisfies the following
    % requirements:

    %PICK_UP_BALL picks up a ball and deposits it in the correct color bin
    % Inputs:
    %   pos: a [1x2] matrix representing the position of the ball in the XY
    %        frame of the robot
    %   color: a string indicating what color bin the ball should be placed
    %          in
    %   z_offset (optional): the z-position at which to begin the straight
    %                        vertical trajectory 
    %   z_ball (optional): the z-posiiton at which to stop the vertical 
    %                      trajectory and grasp the ball
    end % end methods
end % end class 