
% Make sure you can use the functions in the parent directory.
addpath('..');

robot_no_tool = robot_iiwa();
% Make a new robot struct with the tool added to the end effector
tool_length = 100;
robot_w_tool = robot_no_tool;  % Copies by value.
robot_w_tool.home(3,4) = robot.home(3,4) + tool_length;  % Adds tool.
[robot_w_tool.screw, robot_w_tool.bscrew] = calc_screw(robot_w_tool.dof, robot_w_tool.offset, robot_w_tool.axes, robot_w_tool.home);

% Choose starting and ending angles.
% Choose random seed for repeatable randomness.
rng(12345);

%start_angles = [3.60426218545767 1.26563657105162 -3.19483613264846 -6.92175875711803 -1.79462324164591 0.672289765429706 2.96056445057525]';
start_angles = rand(robot.dof, 1) .* (robot.joint_limits(:,2) - robot.joint_limits(:,1)) + robot.joint_limits(:,1);
start_Ts = FK_space(robot_w_tool, start_angles);

% Target with random pose, but definitely reachable.
%target_joint_angles = rand(robot.dof, 1) .* (robot.joint_limits(:,2) - robot.joint_limits(:,1)) + robot.joint_limits(:,1);
%target_Ts  = FK_space(robot_w_tool, target_joint_angles);

% Target with random deviations in joint angles from start.
%target_joint_angles = start_angles + rand(robot.dof, 1) .* 0.05;
%target_Ts  = FK_space(robot_w_tool, target_joint_angles);

% Target with random orientation but only small translation from start.
target_axis = (rand(3, 1) - 0.5);
target_axis = target_axis ./ vecnorm(target_axis);
target_aa = [target_axis; rand(1,1) * 2*pi];
target_translation = trans2translation(start_Ts) + randn(3, 1) * 2;
target_Ts = rottranslation2trans(aa2rot(target_aa), target_translation);

[joint_steps] = move_with_joint_limits(robot_w_tool, start_angles, target_Ts);