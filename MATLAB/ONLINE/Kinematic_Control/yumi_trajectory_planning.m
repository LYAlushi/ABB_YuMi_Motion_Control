%%  Trapezoidal Velocitiy Trajectory Profile Generation
% Author      : L.Y.Alushi
% Date        : 08/2024
% Title       : YuMi Trajectory Planner
% Research    : Motion control with collision avoidance for kinematically redundant manipulator
% Institution : Coventry University
% Supervisor  : K.Al Khudir
%% Begin Function----------------------------------------------------------
function [t_vec, p_L_profile, dp_L_profile, ddp_L_profile, p_R_profile, dp_R_profile, ddp_R_profile] = yumi_trajectory_planning(target_position_L, target_position_R, a_max, v_max)
    %% Initial parameters
    dt = 0.004; % Time step in seconds
    stabilisation_time = 0.5; % 0.5 seconds of stabilisation at the end

    %% Initialise ROS and Get Initial Joint Positions
    try
        rosnode list;
        isROSInitialised = true;
    catch
        isROSInitialised = false;
    end

    if ~isROSInitialised
        rosinit;
    end

    % Subscribe to the /yumi/egm/joint_states topic
    jointStatesSub = rossubscriber('/yumi/egm/joint_states');

    % Receive the latest message from ROS to get the current joint positions
    jointStatesMsg = receive(jointStatesSub);
    jointPositions = jointStatesMsg.Position';

    % Split joint positions for left and right arms
    q_left = jointPositions(1:7);
    q_right = jointPositions(8:14);

    %% Calculate Initial End-Effector Positions
    % Get initial end-effector positions using forward kinematics
    [P_ee_left, P_ee_right, ~, ~] = ForwardKinematics_ABB_IRB14000(q_left, q_right);

    % Calculate the total length of the trajectory
    total_length_L = sqrt((target_position_L(1) - P_ee_left(1))^2 + (target_position_L(2) - P_ee_left(2))^2 + (target_position_L(3) - P_ee_left(3))^2);
    total_length_R = sqrt((target_position_R(1) - P_ee_right(1))^2 + (target_position_R(2) - P_ee_right(2))^2 + (target_position_R(3) - P_ee_right(3))^2);

    % Calculate the time to reach max velocity
    t_accel = v_max / a_max;

    % Calculate the distance covered during acceleration (and deceleration)
    d_accel = (v_max^2) / (2 * a_max);

    % Determine if max velocity is reached
    if total_length_L > 2 * d_accel
        T_L = 2 * t_accel + (total_length_L - 2 * d_accel) / v_max;
    else
        T_L = 2 * sqrt(total_length_L / a_max);
    end

    if total_length_R > 2 * d_accel
        T_R = 2 * t_accel + (total_length_R - 2 * d_accel) / v_max;
    else
        T_R = 2 * sqrt(total_length_R / a_max);
    end

    % Use the maximum time for both arms
    T = max(T_L, T_R);
    % Extend the time vector to include the stabilisation phase
    t_vec = 0:dt:(T + stabilisation_time);
    % Initialise arrays to store the offline computed profiles
    p_L_profile = zeros(length(t_vec), 3);
    dp_L_profile = zeros(length(t_vec), 3);
    ddp_L_profile = zeros(length(t_vec), 3); % For acceleration
    p_R_profile = zeros(length(t_vec), 3);
    dp_R_profile = zeros(length(t_vec), 3);
    ddp_R_profile = zeros(length(t_vec), 3); % For acceleration
    % Offline Trajectory Planning
    for idx = 1:length(t_vec)
        t = t_vec(idx);

        if t <= T
            % Compute desired position, velocity, and acceleration during trajectory
            [p_L, dp_L, ddp_L] = fcn_SpeedProfileCartesian_ABB_IRB14000_YuMi(t, a_max, v_max, P_ee_left(1), P_ee_left(2), P_ee_left(3), target_position_L(1), target_position_L(2), target_position_L(3));
            [p_R, dp_R, ddp_R] = fcn_SpeedProfileCartesian_ABB_IRB14000_YuMi(t, a_max, v_max, P_ee_right(1), P_ee_right(2), P_ee_right(3), target_position_R(1), target_position_R(2), target_position_R(3));
        else
            % Set desired velocities and accelerations to zero during stabilisation
            dp_L = zeros(3,1);
            ddp_L = zeros(3,1);
            dp_R = zeros(3,1);
            ddp_R = zeros(3,1);
            % Keep the position constant
            p_L = target_position_L';
            p_R = target_position_R';
        end

        % Store the computed profiles
        p_L_profile(idx, :) = p_L;
        dp_L_profile(idx, :) = dp_L;
        ddp_L_profile(idx, :) = ddp_L;
        p_R_profile(idx, :) = p_R;
        dp_R_profile(idx, :) = dp_R;
        ddp_R_profile(idx, :) = ddp_R;
    end