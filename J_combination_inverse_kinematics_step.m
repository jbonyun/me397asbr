function [step] = J_combination_inverse_kinematics_step(robot, joint_angles, twist_to_target, lr)
    % Calculate one iteration of IK using a choice of other methods based
    % on the current state.
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
        lr = 0.1;  % Learning rate; how fast we descend along gradient.
    end

    lin_is_big_thresh = 100;
    ang_is_big_thresh = 2;
    iso_is_big_thresh = 1e4;
 
    Jb = J_body(robot, joint_angles);

    % Do we have a large distance to travel?
    %translation = trans2translation(twist2trans(twist_to_target));
    %rpyrotation = rot2rpy(trans2rot(twist2trans(twist_to_target)));
    angdist = norm(twist_to_target(1:3));
    lindist = norm(twist_to_target(4:6));
    %lindist = norm(translation);
    %rpyrotation(rpyrotation > pi) = 2*pi - rpyrotation(rpyrotation > pi);
    %angdist = norm(mod(rpyrotation, pi));
    is_big_distance = (lindist > lin_is_big_thresh) || (angdist > ang_is_big_thresh);

    if is_big_distance
        fprintf('Using J Transpose because far  %.2f  %.2f\n', lindist, angdist);
        step = J_transpose_inverse_kinematics_step(robot, joint_angles, twist_to_target, lr);
    else
        iso = J_isotropy(Jb);
        
        close_to_singular = (iso > iso_is_big_thresh);
        if close_to_singular
            fprintf('Using DLS\n');
            step = DLS_inverse_kinematics_step(robot, joint_angles, twist_to_target, lr);
        else
            fprintf('Using J Inverse\n');
            step = J_inverse_kinematics_step(robot, joint_angles, twist_to_target, lr);
        end
    end


