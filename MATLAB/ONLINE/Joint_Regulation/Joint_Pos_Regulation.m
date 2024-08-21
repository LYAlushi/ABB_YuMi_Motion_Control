%%  Joint Angle regulation for ABB IRB14000 (YuMi)
% Author      : L.Y.Alushi
% Date        : 08/2024
% Title       : Regulation of Joint angle commands
% Research    : Motion control with collision avoidance for kinematically redundant manipulator
% Institution : Coventry University
% Supervisor  : K.Al Khudir
%% Begin Function----------------------------------------------------------
function Joint_Pos_Regulation(desiredPositions)
    % PID controller gains
    Kp = 9.0; % Proportional gain
    Ki = 0.5; % Integral gain
    Kd = 0.01; % Derivative gain

    % Velocity saturation limits Example - should be same as specification
    maxVelocity = 5.0; % Maximum velocity
    minVelocity = -5.0; % Minimum velocity

    % Check if ROS is initialised
    try
        rosnode list;
        isROSInitialised = true;
    catch
        isROSInitialised = false;
    end

    % Initialise ROS if not already initialised
    if ~isROSInitialised
        rosinit;
    end

    % Subscribe to the /yumi/egm/joint_states topic
    jointStatesSub = rossubscriber('/yumi/egm/joint_states');

    % Create a publisher for the /yumi/egm/joint_group_velocity_controller/command topic
    pub = rospublisher('/yumi/egm/joint_group_velocity_controller/command', 'std_msgs/Float64MultiArray');

    % Define the rate at which you want to process the messages (4ms seconds)
    displayRate = 0.004; % seconds

    % Create a loop to continuously read, compute, and publish
    while true
        % Receive the latest message
        jointStatesMsg = receive(jointStatesSub);
        
        % Get the current positions
        jointPositions = jointStatesMsg.Position';
        
        % Split joint positions for left and right arms
        q_left = jointPositions(1:7);
        q_right = jointPositions(8:14);
        
        % Compute forward kinematics
        [P_ee_left, P_ee_right, Jacobian_left, Jacobian_right] = ForwardKinematics_ABB_IRB14000_ArmsOnly(q_left, q_right);
        
        % Display forward kinematics results
        disp('End-Effector Positions (Left Arm, Right Arm):');
        disp(P_ee_left);
        disp(P_ee_right);
        
        disp('Jacobians (Left Arm, Right Arm):');
        disp(Jacobian_left);
        disp(Jacobian_right);
        
        % Call the PID control function
        controlOutput = pidControl(jointPositions, desiredPositions, Kp, Ki, Kd, displayRate);
        
        % Saturate the control output to within the velocity limits
        controlOutput(controlOutput > maxVelocity) = maxVelocity;
        controlOutput(controlOutput < minVelocity) = minVelocity;
        
        % Create and populate the message to be published
        msg = rosmessage(pub);
        msg.Data = controlOutput;
        
        % Publish the message
        send(pub, msg);
        
        % Display the current state and control output
        disp('Current Positions:');
        disp(jointPositions);
        disp('Desired Positions:');
        disp(desiredPositions);
        disp('Control Output (Velocities):');
        disp(controlOutput);
        
        % Pause for the defined display rate
        pause(displayRate);
    end
end
%% End Function----------------------------------------------------------
