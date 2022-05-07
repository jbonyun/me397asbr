
addpath('..');

robot_no_tool = robot_iiwa();
tool_length = 100;
% Make a new robot struct with the tool added to the end effector
robot_w_tool = robot_no_tool;
robot_w_tool.home(3,4) = robot.home(3,4) + tool_length;
[robot_w_tool.screw, robot_w_tool.bscrew] = calc_screw(robot_w_tool.dof, robot_w_tool.offset, robot_w_tool.axes, robot_w_tool.home);

start_angles = [3.60426218545767 1.26563657105162 -3.19483613264846 -6.92175875711803 -1.79462324164591 0.672289765429706 2.96056445057525]';
target_joint_angles = rand(robot.dof, 1) .* (robot.joint_limits(:,2) - robot.joint_limits(:,1)) + robot.joint_limits(:,1);
target_Ts  = FK_space(robot_w_tool, target_joint_angles);

[joint_steps] = move_with_joint_limits(robot_w_tool, start_angles, target_Ts);