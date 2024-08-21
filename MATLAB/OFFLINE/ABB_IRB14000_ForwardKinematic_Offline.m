%% Forward Kinematics for ABB IRB14000 YuMi -------------------------------
% Author      : L.Y.Alushi
% Date        : 08/2024
% Title       : Offline Computation of ABB IRB14000 YuMi Forward Kinematics
% Description : Homogenous Transformation Matrices product from base
%               reference frame to end-effectors [dual-arm] shared base of 
%               robot for base of arms. 
%               Also, the Computation of Symbolic Jacobian Matrices.
% Research    : Motion Control with Collision Avoidance for kinematically
%               redundant manipulator
% Institution : Coventry University
% Supervisor  : Dr. K. Al Khudir
%% Begin Script -----------------------------------------------------------
%% Initialisation
% Define symbolic joint angles for the 7 joints of both arms
syms q1L q2L q3L q4L q5L q6L q7L q1R q2R q3R q4R q5R q6R q7R real;
% Function to create a rotation matrix from RPY angles
rpy_to_rotm = @(rpy) eul2rotm(flip(rpy), 'ZYX');
% Function to create a translation matrix
transl = @(xyz) [eye(3), xyz(:); 0 0 0 1];
% Function to create a rotation matrix around the z-axis (symbolically)
rotz_sym = @(theta) [cos(theta), -sin(theta), 0; sin(theta), cos(theta), 0; 0, 0, 1];
% Function to create a 4x4 homogeneous transformation matrix from RPY and XYZ
T = @(xyz, rpy) transl(xyz) * [rpy_to_rotm(rpy), zeros(3,1); 0 0 0 1];
%% Left Arm
% Joint 1: from yumi_body to yumi_link_1_l
T01L = T([0.05355, 0.07250, 0.41492], [0.9781, -0.5716, 2.3180]) * ...
       [rotz_sym(q1L), [0;0;0]; 0 0 0 1];
% Joint 2: from yumi_link_1_l to yumi_link_2_l
T12L = T([0.03, 0, 0.1], [1.571, 0, 0]) * ...
       [rotz_sym(q2L), [0;0;0]; 0 0 0 1];
% Joint 3: from yumi_link_2_l to yumi_link_3_l
T23L = T([-0.03, 0.17283, 0], [-1.571, 0, 0]) * ...
       [rotz_sym(q3L), [0;0;0]; 0 0 0 1];
% Joint 4: from yumi_link_3_l to yumi_link_4_l
T34L = T([-0.04188, 0, 0.07873], [1.571, -1.571, 0]) * ...
       [rotz_sym(q4L), [0;0;0]; 0 0 0 1];
% Joint 5: from yumi_link_4_l to yumi_link_5_l
T45L = T([0.0405, 0.16461, 0], [-1.571, 0, 0]) * ...
       [rotz_sym(q5L), [0;0;0]; 0 0 0 1];
% Joint 6: from yumi_link_5_l to yumi_link_6_l
T56L = T([-0.027, 0, 0.10039], [1.571, 0, 0]) * ...
       [rotz_sym(q6L), [0;0;0]; 0 0 0 1];
% Joint 7: from yumi_link_6_l to yumi_link_7_l
T67L = T([0.027, 0.029, 0], [-1.571, 0, 0]) * ...
       [rotz_sym(q7L), [0;0;0]; 0 0 0 1];
% End-Effector (Gripper base): from yumi_link_7_l to gripper_l_base
T78L = T([0, 0, 0.007], [0, 0, 3.1416]);
% Compute the total transformation matrix for the left arm
T_total_L = (T01L * T12L * T23L * T34L * T45L * T56L * T67L * T78L);
% Extract the position of the left end effector
position_end_effector_L = T_total_L(1:3, 4);
%% Right Arm
% Joint 1: from yumi_body to yumi_link_1_r
T01R = T([0.05355, -0.0725, 0.41492], [-0.9781, -0.5682, -2.3180]) * ...
       [rotz_sym(q1R), [0;0;0]; 0 0 0 1];
% Joint 2: from yumi_link_1_r to yumi_link_2_r
T12R = T([0.03, 0, 0.1], [1.571, 0, 0]) * ...
       [rotz_sym(q2R), [0;0;0]; 0 0 0 1];
% Joint 3: from yumi_link_2_r to yumi_link_3_r
T23R = T([-0.03, 0.17283, 0], [-1.571, 0, 0]) * ...
       [rotz_sym(q3R), [0;0;0]; 0 0 0 1];
% Joint 4: from yumi_link_3_r to yumi_link_4_r
T34R = T([-0.04188, 0, 0.07873], [1.571, -1.571, 0]) * ...
       [rotz_sym(q4R), [0;0;0]; 0 0 0 1];
% Joint 5: from yumi_link_4_r to yumi_link_5_r
T45R = T([0.0405, 0.16461, 0], [-1.571, 0, 0]) * ...
       [rotz_sym(q5R), [0;0;0]; 0 0 0 1];
% Joint 6: from yumi_link_5_r to yumi_link_6_r
T56R = T([-0.027, 0, 0.10039], [1.571, 0, 0]) * ...
       [rotz_sym(q6R), [0;0;0]; 0 0 0 1];
% Joint 7: from yumi_link_6_r to yumi_link_7_r
T67R = T([0.027, 0.029, 0], [-1.571, 0, 0]) * ...
       [rotz_sym(q7R), [0;0;0]; 0 0 0 1];
% End-Effector (Gripper base): from yumi_link_7_r to gripper_r_base
T78R = T([0, 0, 0.007], [0, 0, 3.1416]);
% Compute the total transformation matrix for the right arm
T_total_R = (T01R * T12R * T23R * T34R * T45R * T56R * T67R * T78R);
% Extract the position of the right end effector
position_end_effector_R = T_total_R(1:3, 4);
%% Computation of Jacobian matrices for both arms
Jacobian_P_ee_L = jacobian(position_end_effector_L, [q1L, q2L, q3L, q4L, q5L, q6L, q7L]);
Jacobian_P_ee_R = jacobian(position_end_effector_R, [q1R, q2R, q3R, q4R, q5R, q6R, q7R]);
%% Display the positions of both end effectors and Jacobians
disp('Left Arm End-Effector Position (symbolic):');
disp(position_end_effector_L);
disp('Right Arm End-Effector Position (symbolic):');
disp(position_end_effector_R);
disp('Jacobian for Left Arm End-Effector (symbolic):');
disp(Jacobian_P_ee_L);
disp('Jacobian for Right Arm End-Effector (symbolic):');
disp(Jacobian_P_ee_R);
%% End Script -------------------------------------------------------------