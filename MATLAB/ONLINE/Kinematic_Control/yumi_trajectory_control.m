%%  Motion Task Function for ABB IRB14000 (YuMi) Collision Avoidance Static
% Author      : L.Y.Alushi
% Date        : 08/2024
% Title       : General trajectory control YuMi
% Research    : Motion control with collision avoidance for kinematically redundant manipulator
% Institution : Coventry University
% Supervisor  : K.Al Khudir
%% Begin Function----------------------------------------------------------
function yumi_trajectory_control(t_vec, p_L_profile, dp_L_profile, p_R_profile, dp_R_profile, kp, v_max)
    % Online Control Loop Using Precomputed Trajectories
    % Initialize arrays to store data for plotting
    pos_L = zeros(length(t_vec), 3);
    pos_R = zeros(length(t_vec), 3);
    vel_L = zeros(length(t_vec), 7);
    vel_R = zeros(length(t_vec), 7);
    err_L = zeros(length(t_vec), 3);
    err_R = zeros(length(t_vec), 3);
    joint_pos_L = zeros(length(t_vec), 7);
    joint_pos_R = zeros(length(t_vec), 7);

    % Initialize ROS if not already initialized
    try
        rosnode list;
        isROSInitialized = true;
    catch
        isROSInitialized = false;
    end
    if ~isROSInitialized
        rosinit;
    end
    % Subscribe to the /yumi/egm/joint_states topic
    jointStatesSub = rossubscriber('/yumi/egm/joint_states');
    % Create a publisher for the /yumi/egm/joint_group_velocity_controller/command topic
    pub = rospublisher('/yumi/egm/joint_group_velocity_controller/command', 'std_msgs/Float64MultiArray');

    for idx = 1:length(t_vec)
        % Use the precomputed desired positions and velocities
        p_L = p_L_profile(idx, :)';
        dp_L = dp_L_profile(idx, :)';
        p_R = p_R_profile(idx, :)';
        dp_R = dp_R_profile(idx, :)';
        % Receive the latest message from ROS
        jointStatesMsg = receive(jointStatesSub);
        jointPositions = jointStatesMsg.Position';
        % Split joint positions for left and right arms
        q_left = jointPositions(1:7);
        q_right = jointPositions(8:14);
        % Store the measured joint positions
        joint_pos_L(idx, :) = q_left;
        joint_pos_R(idx, :) = q_right;
        % Direct kinematics to get current end-effector positions
        [current_p_L, current_p_R, Jacobian_L, Jacobian_R] = ForwardKinematics_ABB_IRB14000(q_left, q_right);
        % Compute position error
        error_L = p_L - current_p_L;
        error_R = p_R - current_p_R;
        % Store errors for plotting
        err_L(idx, :) = error_L;
        err_R(idx, :) = error_R;
        % Compute joint velocities using resolved-rate control
        joint_velocities_L = pinv(Jacobian_L) * (dp_L + (kp * error_L));
        joint_velocities_R = pinv(Jacobian_R) * (dp_R + (kp * error_R));
        % Saturate the control output to within the velocity limits
        joint_velocities_L = min(max(joint_velocities_L, -v_max), v_max);
        joint_velocities_R = min(max(joint_velocities_R, -v_max), v_max);
        % Create and populate the message to be published
        msg = rosmessage(pub);
        msg.Data = [joint_velocities_L; joint_velocities_R];
        % Publish the message
        send(pub, msg);
        % Store data for plotting
        pos_L(idx, :) = current_p_L;
        pos_R(idx, :) = current_p_R;
        vel_L(idx, :) = joint_velocities_L';
        vel_R(idx, :) = joint_velocities_R';
    end
%% End Function----------------------------------------------------------
