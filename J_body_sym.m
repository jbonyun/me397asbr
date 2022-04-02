function [J] = J_body_sym(robot, joint_angles)
    % Calculates the jacobian in the body frame.
    % Inputs:
    %   robot: struct with robot description
    %            field dof: integer number of degrees of freedom
    %            field screw: 6xdof matrix of screw vectors for each joint
    %   joint_angles: angles of each joint, in rad
    % Outputs:
    %   J: 6xdof Jacobian matrix

    % On behalf of the Sun/Bonyun team for ME397 ASBR, Spring 2022.
    % Source: Alambeigi, F. ASBR Lecture Notes. 2022, W7-L1 p. 10.

    % First column is just the screw for first joint
    J(:, robot.dof) = robot.bscrew(:, robot.dof);
    % Keep a running product of the forward kinematics from end to start
    prod_expon = expm_sym(-robot.bscrew(:, robot.dof),joint_angles(robot.dof));
    for i = (robot.dof-1):-1:1
        % Calculate this column of J
        J(:, i) = adjoint_sym(prod_expon) * robot.bscrew(:, i);
        % Update running product of FK of joints we have passed
        prod_expon = prod_expon * expm_sym(-robot.bscrew(:, i),joint_angles(i));
    end
