function [step] = DLS_inverse_kinematics_step(robot, joint_angles, twist_to_target, lr, k)
    % Calculate one iteration of J-inverse IK with DLS to handle singularities.
    % 
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
        k = 0.01;  % Finite difference of angle for manipulability slope.
    end

    Jb = J_body(robot, joint_angles);
    J_star = Jb' * inv(Jb*Jb' + k^2 * eye(6));
    step = J_star * twist_to_target * lr;
