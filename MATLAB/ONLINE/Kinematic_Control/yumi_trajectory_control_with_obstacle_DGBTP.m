%%  Motion Task Function for ABB IRB14000 (YuMi) Collision Avoidance DG-TP
% Author      : L.Y.Alushi
% Date        : 08/2024
% Title       : Static  collision avoidance trajectory control YuMi DG-TP
% Research    : Motion control with collision avoidance for kinematically redundant manipulator
% Institution : Coventry University
% Supervisor  : K.Al Khudir
%% Begin Function----------------------------------------------------------
function yumi_trajectory_control_with_obstacle_DGBTP(t_vec, p_L_profile, dp_L_profile, p_R_profile, dp_R_profile, v_max)
    Obst_L = [0.304593198952846, 0.349533463478006, 0.352922717301088];
    Obst_R = [0.0566167701468179, -0.217701407570880, 0.229915871405198];
    threshold_L = 0.05;
    threshold_R = 0.05;
    % Define max positional gains
    max_kp_L = [7, 0, 0; 0, 7, 0; 0, 0, 7];
    max_kp_R = [7, 0, 0; 0, 7, 0; 0, 0, 7];
    % Online Control Loop Using Precomputed Trajectories
    % Initialise arrays to store data 
    pos_L = zeros(length(t_vec), 3);
    pos_R = zeros(length(t_vec), 3);
    vel_L = zeros(length(t_vec), 7);
    vel_R = zeros(length(t_vec), 7);
    err_L = zeros(length(t_vec), 3);
    err_R = zeros(length(t_vec), 3);
    joint_pos_L = zeros(length(t_vec), 7);
    joint_pos_R = zeros(length(t_vec), 7);
    distance_to_obst_L = zeros(length(t_vec), 1);  % To store distance to the obstacle for left arm (planned)
    real_distance_to_obst_L = zeros(length(t_vec), 1);  % To store distance to the obstacle for left arm (actual)
    distance_to_obst_R = zeros(length(t_vec), 1);  % To store distance to the obstacle for right arm (planned)
    real_distance_to_obst_R = zeros(length(t_vec), 1);  % To store distance to the obstacle for right arm (actual)

    % Initialise ROS if not already initialised
    try
        rosnode list;
        isROSInitialized = true;
    catch
        isROSInitialized = false;
    end

    if ~isROSInitialized
        rosinit;
    end

    % Subscribe to the /yumi/egm/joint_states topic (same as before)
    jointStatesSub = rossubscriber('/yumi/egm/joint_states');
    pub = rospublisher('/yumi/egm/joint_group_velocity_controller/command', 'std_msgs/Float64MultiArray');

    for idx = 1:length(t_vec)
        % Use the precomputed desired positions and velocities
        p_L = p_L_profile(idx, :)';
        dp_L = dp_L_profile(idx, :)';
        p_R = p_R_profile(idx, :)';
        dp_R = dp_R_profile(idx, :)';

        % Calculate the current distance to the obstacle for left arm (planned)
        distance_obst_L = norm(p_L - Obst_L');
        distance_to_obst_L(idx) = distance_obst_L;
        
        % ROS message handling and publishing (same as before)
        jointStatesMsg = receive(jointStatesSub);
        jointPositions = jointStatesMsg.Position';

        q_left = jointPositions(1:7);
        q_right = jointPositions(8:14);

        joint_pos_L(idx, :) = q_left;
        joint_pos_R(idx, :) = q_right;

        [current_p_L, current_p_R, Jacobian_L, Jacobian_R] = ForwardKinematics_ABB_IRB14000(q_left, q_right);

        % Calculate the current distance to the obstacle for left arm (actual)
        real_distance_obst_L = norm(current_p_L - Obst_L');
        real_distance_to_obst_L(idx) = real_distance_obst_L;

        % Apply a scaling factor based on the distance to the obstacle
        scale_L = min(1, max(0, (distance_obst_L - threshold_L) / threshold_L));
        kp_L = scale_L * max_kp_L;

        % Check for obstacle avoidance condition for left arm
        if distance_obst_L < threshold_L
            % Define potential field parameters for left arm
            survaliance_L = 0.1; % Danger threshold
            v_max_L = 0.4;
            rho_L = survaliance_L;
            gamma_L = 3; % More sharp avoidance
            const_L = 1.2; % Shape parameter

            % Compute repulsive velocity for left arm
            r_magnitude_L = v_max_L / (1 + exp(distance_obst_L * (const_L/rho_L) * gamma_L - gamma_L));
            repulsive_direction_L = (p_L - Obst_L') / distance_obst_L;
            repulsive_vector_L = r_magnitude_L * repulsive_direction_L;

            % Add repulsive vector to the desired velocity for left arm
            dp_L = dp_L + repulsive_vector_L;

            % Optionally set control gains to zero to focus on avoidance for left arm
            % kp_L = zeros(3); % Uncomment if you want to zero out the gains completely when avoiding
        end

        % Calculate the current distance to the obstacle for right arm (planned)
        distance_obst_R = norm(p_R - Obst_R');
        distance_to_obst_R(idx) = distance_obst_R;

        % Calculate the current distance to the obstacle for right arm (actual)
        real_distance_obst_R = norm(current_p_R - Obst_R');
        real_distance_to_obst_R(idx) = real_distance_obst_R;

        % Apply a scaling factor based on the distance to the obstacle
        scale_R = min(1, max(0, (distance_obst_R - threshold_R) / threshold_R));
        kp_R = scale_R * max_kp_R;

        % Check for obstacle avoidance condition for right arm
        if distance_obst_R < threshold_R
            % Define potential field parameters for right arm
            survaliance_R = 0.1; % Danger threshold
            v_max_R = 0.4;
            rho_R = survaliance_R;
            gamma_R = 3; % More sharp avoidance
            const_R = 1.2; % Shape parameter

            % Compute repulsive velocity for right arm
            r_magnitude_R = v_max_R / (1 + exp(distance_obst_R * (const_R/rho_R) * gamma_R - gamma_R));
            repulsive_direction_R = (p_R - Obst_R') / distance_obst_R;
            repulsive_vector_R = r_magnitude_R * repulsive_direction_R;

            % Add repulsive vector to the desired velocity for right arm
            dp_R = dp_R + repulsive_vector_R;

            % Optionally set control gains to zero to focus on avoidance for right arm
            % kp_R = zeros(3); % Uncomment if you want to zero out the gains completely when avoiding
        end

        error_L = p_L - current_p_L;
        error_R = p_R - current_p_R;

        err_L(idx, :) = error_L;
        err_R(idx, :) = error_R;

        joint_velocities_L = pinv(Jacobian_L) * (dp_L + (kp_L * error_L));
        joint_velocities_R = pinv(Jacobian_R) * (dp_R + (kp_R * error_R));

        joint_velocities_L = min(max(joint_velocities_L, -v_max), v_max);
        joint_velocities_R = min(max(joint_velocities_R, -v_max), v_max);

        msg = rosmessage(pub);
        msg.Data = [joint_velocities_L; joint_velocities_R];

        send(pub, msg);

        pos_L(idx, :) = current_p_L;
        pos_R(idx, :) = current_p_R;
        vel_L(idx, :) = joint_velocities_L';
        vel_R(idx, :) = joint_velocities_R';
    end
%% End Function------------------------------------------------------------
