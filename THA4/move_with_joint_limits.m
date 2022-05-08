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
    startTs = FK_space(robot, start_angles);
    % Get t vector (translation of tip wrt space frame).
    t = trans2translation(startTs);
    % Get vector pgoal from the destination location.
    pgoal = trans2translation(destTs);

    Js = J_space(robot, start_angles);
    Jalpha = Js(1:3, :);
    Jeps = Js(4:6, :);

    % Joint limit bounds
    qL = robot.joint_limits(:, 1);
    qU = robot.joint_limits(:, 2);
    qLA = -eye(7);
    qLb = start_angles - qL;
    qUA = eye(7);
    qUb = qU - start_angles;

    % Set up the linear least squares problem.
    %   C,d s.t. we minimize Cx - d
    %   A,b s.t. we require Ax <= b
    %x = lsqlin(C,d,A,b);
    C = -skewsym(t) * Jalpha + Jeps;
    d = pgoal - t;
    A = [];
    b = [];
    %A = -skewsym(t) * Jalpha + Jeps;
    %b = 3 + pgoal - t;
    A = [A; qLA; qUA];
    b = [b; qLb; qUb];
    [dq, resnorm, residual, exitflag, output, lambda] = lsqlin(C, d, A, b);