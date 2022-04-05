function [J] = J_space(robot, joint_angles, varargin)
    % Calculates the jacobian in the space frame.
    % Inputs:
    %   robot: struct with robot description
    %            field dof: integer number of degrees of freedom
    %            field screw: 6xdof matrix of screw vectors for each joint
    %   joint_angles: angles of each joint, in rad
    % Outputs:
    %   J: 6xdof Jacobian matrix
    % Jeff Bonyun (jb79332), jbonyun@utexas.edu, 20220324
    % On behalf of the Sun/Bonyun team for ME397 ASBR, Spring 2022.
    % Source: Alambeigi, F. ASBR Lecture Notes. 2022, W7-L1 p. 7.

    p = inputParser;
    addRequired(p, 'robot'); 
    addRequired(p, 'joint_angles'); 
    addParameter(p, 'JointNum', nan);
    parse(p, robot, joint_angles, varargin{:});
    args = p.Results;

    if isnan(args.JointNum)
        maxn = robot.dof;
    else
        maxn = args.JointNum;
    end

    % First column is just the screw for first joint
    J(:, 1) = robot.screw(:, 1);
    % Keep a running product of the forward kinematics
    prod_expon = expm(skewsym(robot.screw(:, 1)) * joint_angles(1));
    for i = 2:maxn
        % Calculate this column of J
        J(:, i) = adjoint_transform(prod_expon) * robot.screw(:, i);
        % Update running product of FK of joints we have passed
        prod_expon = prod_expon * expm(skewsym(robot.screw(:, i)) * joint_angles(i));
    end
