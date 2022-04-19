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

    % From paper: Introduction to Inverse Kinematics with Jacobian
    % Transpose, Pseudoinverse, and Damped Least Squares methods. Buss.
    % 2009. p 7.
    % Assuming twist can be used in place of e, which is cartesian distance
    % but the paper doesn't address orientation, so it's a little vague
    % about how to handle 6 dof.
    alpha = dot(twist_to_target, Jb * Jb' * twist_to_target) / dot(Jb * Jb' * twist_to_target, Jb * Jb' * twist_to_target);
    lr = lr * alpha;
    fprintf('alpha = %f\n', alpha);

    K = diag([2500 2500 2500 1 1 1]);

    step = Jb' * K * twist_to_target * lr;



