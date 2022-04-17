function [step] = J_transpose_inverse_kinematics_step(robot, joint_angles, twist_to_target, lr)
    % Calculate one iteration of J-transpose IK.
    % 
    % Inputs:
    %   robot: struct with robot description
    %   joint_angles: nx1 joint angles.
    %   twist_to_target: 6x1 twist from current position to target position.
    % Output:
    %    step: nx1 joint angle change in radians for this step
    % On behalf of the Sun/Bonyun team for ME397 ASBR, Spring 2022.
    % Source: Alambeigi, F. ASBR Lecture Notes. 2022, W10-L1 p. 9.

    % Some constants for the algorithm.
    if nargin < 4
        lr = 1e-6;  % Learning rate; how fast we descend along gradient.
    end

    % Calculate step
    Jb = J_body(robot, joint_angles);
    step = Jb' * twist_to_target * lr;



