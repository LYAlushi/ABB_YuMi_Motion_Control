function yumi_where_am_I()
    %% Initialise ROS if not already initialised
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

    % Receive the latest message from ROS
    jointStatesMsg = receive(jointStatesSub);
    jointPositions = jointStatesMsg.Position';

    % Split joint positions for left and right arms
    q_left = jointPositions(1:7)
    q_right = jointPositions(8:14)

    % Direct kinematics to get current end-effector positions
    [current_p_L, current_p_R, ~, ~] = ForwardKinematics_ABB_IRB14000(q_left, q_right);

    % Display the current end-effector positions
    fprintf('Current End-Effector Position (Left Arm):\n');
    fprintf('X: %.4f, Y: %.4f, Z: %.4f\n', current_p_L(1), current_p_L(2), current_p_L(3));
    fprintf('Current End-Effector Position (Right Arm):\n');
    fprintf('X: %.4f, Y: %.4f, Z: %.4f\n', current_p_R(1), current_p_R(2), current_p_R(3));
end
