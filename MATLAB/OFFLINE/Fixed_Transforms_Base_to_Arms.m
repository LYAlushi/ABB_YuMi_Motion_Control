%% Fixed Transforms Dual-arm ABB IRB14000 YuMi ----------------------------
% Author      : L.Y.Alushi
% Date        : 08/2024
% Title       : Visualise Fixed Transforms for base of arms to rob base
% Description : Fixed additional transforms for shared coordinate frame
% Research    : Motion Control with Collision Avoidance for kinematically
%               redundant manipulator
% Institution : Coventry University
% Supervisor  : Dr. K. Al Khudir
%% Begin Script -----------------------------------------------------------
% Define the rotation and translation components for the left arm's base frame
R_left = [-0.5716, -0.1048, 0.8138; 
               0.617, -0.7088, 0.342; 
               0.541, 0.6976, 0.4698];
T_left = [0.0536; 0.0725; 0.4149];
% Construct the homogeneous transformation matrix for the left arm
T_left_arm = [R_left, T_left; 
                  0, 0, 0, 1];
% Define the rotation and translation components for the right arm's base frame
R_right = [-0.5716, 0.1071, 0.8138; 
               -0.6198, -0.7068, -0.3421; 
                0.541, -0.6976, 0.4698];
T_right = [0.0536; -0.0725; 0.4149];
% Construct the homogeneous transformation matrix for the right arm
T_right_arm = [R_right, T_right; 
                   0, 0, 0, 1];
% Visualise the frames
figure;
hold on;
axis equal;
grid on;
% Plot the global reference frame
plot_frame(eye(4), 0.1);
% Plot the left arm frame
plot_frame(T_left_arm, 0.1);
% Plot the right arm frame
plot_frame(T_right_arm, 0.1);
% Labels
text(0, 0, 0, 'Base', 'FontSize', 10, 'Color', 'k', 'FontName', 'Times New Roman');
text(T_left(1), T_left(2), T_left(3), 'Left Arm', 'FontSize', 10, 'Color', 'r', 'FontName', 'Times New Roman');
text(T_right(1), T_right(2), T_right(3), 'Right Arm', 'FontSize', 10, 'Color', 'b', 'FontName', 'Times New Roman');
xlabel('X', 'FontName', 'Times New Roman');
ylabel('Y', 'FontName', 'Times New Roman');
zlabel('Z', 'FontName', 'Times New Roman');
title('Fixed Transformations from YuMi Body to Base of Each Manipulator Arm', 'FontName', 'Times New Roman');
view(3); % Enable 3D view
hold off;
% Plotting function
function plot_frame(T, scale)
    % Extract rotation matrix and translation vector
    R = T(1:3, 1:3);
    t = T(1:3, 4);
    
    % Plot origin
    plot3(t(1), t(2), t(3), 'ko');
    
    % Plot x-axis (red)
    quiver3(t(1), t(2), t(3), scale * R(1, 1), scale * R(2, 1), scale * R(3, 1), 'r', 'LineWidth', 2);
    % Plot y-axis (green)
    quiver3(t(1), t(2), t(3), scale * R(1, 2), scale * R(2, 2), scale * R(3, 2), 'g', 'LineWidth', 2);
    % Plot z-axis (blue)
    quiver3(t(1), t(2), t(3), scale * R(1, 3), scale * R(2, 3), scale * R(3, 3), 'b', 'LineWidth', 2);
end
%% End Script -------------------------------------------------------------