function [config] = getrobotconfig(robot_model, joint_angles)
    % Returns a robotics toolbox configuration for the robot_model given
    % Includes the provided joint angles
    config = homeConfiguration(robot_model);
    num2cell(joint_angles);
    [config.JointPosition] = ans{:};