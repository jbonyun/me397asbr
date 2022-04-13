function [step] = J_transpose_inverse_kinematics_step(robot, joint_angles, twist_to_target, dest_T, lr, posdefK)
    % Calculate one iteration of J-transpose IK.
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
    if nargin < 5
        lr = 1e-6;  % Learning rate; how fast we descend along gradient. Notes say "1".
    end
    if nargin < 6
        % TODO: why is this the right answer? What are the requirements on
        %       K? Need to read the notes again.
        V = diag([1 1 1 2 2 2]);
        U = eye(6);
        posdefK= U * V *U';
    end

    % TODO: Why are we using trans2vector? Does twist achieve the same
    % thing? Or is something magical about trans2vector? Need to read the
    % notes again.

    Xd = trans2vector(dest_T);  % Xd is the trans2vector of the destination transform

    % Calculate step
    Jb = J_body(robot,joint_angles);
    Xe = trans2vector(FK_space(robot,joint_angles));
    err = Xd - Xe;
    step = transpose(Jb) * posdefK * err * lr;
 
