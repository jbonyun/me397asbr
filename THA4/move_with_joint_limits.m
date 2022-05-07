function [joint_steps] = move_with_joint_limits(robot, start_angles, destTs)
    % Calculates a path from startT to destT.
    % Respects the joint limits in the robot struct.
    % Input:
    %  robot:       struct of robot information
    %               dof: scalar number of joints
    %               joint_limits: dofx2 matrix with min and max
    %                             angles for each joint (rad).
    % start_angles: 1xk vector of starting joint angles.
    % destTs:        4x4 target transformation matrix in space frame (which
    %               is the robot root).
    % Output:
    %  joint_steps: mxk matrix of joint angles along the path to destT
    %               m is a number of steps
    %               k is the number of joints

     % Apply FK to find end effector position in space frame.
     % This is the vector t from the notes.
    startTs = FK_space(robot, start_angles);
    