function [step] = redundancy_resolution_inverse_kinematics_step(robot, joint_angles, twist_to_target, lr, k0, deltaq)
    % Calculate one iteration of J-inverse IK with redundancy resolution.
    % Has a secondary objective of maximizing the manipulability ellispoid
    % volume, while it seeks to achieve the desired destination.
    % Inputs:
    %   robot: struct with robot description
    %   joint_angles: nx1 joint angles.
    %   twist_to_target: 6x1 twist from current position to target position.
    % Output:
    %    step: nx1 joint angle change in radians for this step
    % Jeff Bonyun (jb79332), jbonyun@utexas.edu, 20220413
    % On behalf of the Sun/Bonyun team for ME397 ASBR, Spring 2022.
    % Source: Alambeigi, F. ASBR Lecture Notes. 2022, W10-L1 p. 9.

    % Suppress annoying warning about inv
    %#ok<*MINV> 

    % Some constants for the algorithm.
    if nargin < 4
        lr = 0.5;  % Learning rate; how fast we descend along gradient. Notes say "1".
    end
    if nargin < 5
        k0 = 1e-7;  % Scaling factor for manipulability slope.
    end
    if nargin < 6
        deltaq = 0.01;  % Finite difference of angle for manipulability slope.
    end


   Jb = J_body(robot, joint_angles);
    % Calculate partial derivative of manipulability wrt joints
    w0 = sqrt(det(Jb*Jb'));
    q0_dot = nan(1, 7);
    for i=1:7
        q0 = joint_angles;
        q0(i) = q0(i) + deltaq;
        Ji = J_body(robot, q0);
        wi = sqrt(det(Ji*Ji'));
        q0_dot(i) = (wi - w0) / deltaq;
    end
    % Calculate step
    J_daggr = dagger_J(Jb,7,6);
    step = J_daggr * twist_to_target * lr + (eye(7,7) - J_daggr *Jb) * k0 * q0_dot';
