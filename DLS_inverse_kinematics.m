function joint_angles = DLS_inverse_kinematics(robot, guess_joint_angles, Tsd)
    % Solve inverse kinematics using DLS to avoid singularities.
    % Inputs:
    %   robot: struct with robot description
    %   guess_joint_angles: nx1 starting guess for joint angles.
    %   Tsd: 4x4 desired configuration, as a transformation.
    % Output:
    %    joint_angles: nx1 joint angles in radians that will acheive the
    %                  requested transformation.
    % Jeff Bonyun (jb79332), jbonyun@utexas.edu, 20220406
    % On behalf of the Sun/Bonyun team for ME397 ASBR, Spring 2022.
    % Source: Alambeigi, F. ASBR Lecture Notes. 2022, W10-L1 p. 9.

    % Suppress annoying warning about inv
    %#ok<*MINV> 

    % Some constants for the algorithm.
    lr = 1;  % Learning rate; how fast we descend along gradient. Notes say "1".
    angular_thresh = 0.001;  % When to stop the search.
    linear_thresh = 0.01;  % When to stop the search.
    k = 0.01;

    % Helper lambda for getting the twist.
    get_twist = @(joint_angles) get_twist_ex(robot, joint_angles, Tsd);

    % Start with the initial guess.
    joint_angles = guess_joint_angles;

    iter = 0;
    twist_b = get_twist(joint_angles);
    cond_num = J_condition(J_body(robot, joint_angles));
    fprintf('Iter %d  StepNorm: %8f  ErrAng %9f  ErrLin %9f  Cond %12.2f  Angles %s\n', iter, nan, norm(twist_b(1:3)), norm(twist_b(4:6)), cond_num, mat2str(joint_angles'));
    while norm(twist_b(1:3)) > angular_thresh || norm(twist_b(4:6)) > linear_thresh
        iter = iter + 1;
        % Calculate step
        Jb = J_body(robot, joint_angles);
        J_star = Jb' * inv(Jb*Jb' + k^2 * eye(6));
        step = J_star * twist_b * lr;
        % Update the solution
        joint_angles = mod(joint_angles + step, 2*pi);
        % Prepare state for next iteration
        twist_b = get_twist(joint_angles);
        % Report on progress
        if mod(iter, 1) == 0
            cond_num = J_condition(Jb);
            fprintf('Iter %d  StepNorm: %8f  ErrAng %9f  ErrLin %9f  Cond %12.2f  Angles %s\n', iter, norm(step), norm(twist_b(1:3)), norm(twist_b(4:6)), cond_num, mat2str(joint_angles'));
        end
    end
end

function twist = get_twist_ex(robot, joint_angles, Tsd)
    trans = inv(FK_space(robot, joint_angles)) * Tsd;
    [screw, theta] = trans2screw(trans);
    twist = screw2twist(screw, theta);
end