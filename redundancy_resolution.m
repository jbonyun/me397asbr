function [joint_angles, iter_errang, iter_errlin, iter_cond, iter_stepnorm] = redundancy_resolution(robot, guess_joint_angles, Tsd)
    % Solve inverse kinematics with redundancy resolution via maximizing
    % manipulability.
    % Inputs:
    %   robot: struct with robot description
    %   guess_joint_angles: nx1 starting guess for joint angles.
    %   Tsd: 4x4 desired configuration, as a transformation.
    % Output:
    %    joint_angles: nx1 joint angles in radians that will acheive the
    %                  requested transformation.
    %    iter_*: kx1 iteration-by-iteration progress of metrics.
    % Jeff Bonyun (jb79332), jbonyun@utexas.edu, 20220407
    % On behalf of the Sun/Bonyun team for ME397 ASBR, Spring 2022.
    % Source: Alambeigi, F. ASBR Lecture Notes. 2022, W10-L1 p. 9.

    % Suppress annoying warning about inv
    %#ok<*MINV> 

    % Some constants for the algorithm.
    lr = 0.5;  % Learning rate; how fast we descend along gradient. Notes imply "1". Smaller can be smoother, but more iterations.
    deltaq = 0.01;  % Finite difference of angle for manipulability slope.
    k0 = 1e-7;  % Scaling factor for manipulability slope.
    angular_thresh = 0.001;  % When to stop the search.
    linear_thresh = 0.01;  % When to stop the search.
    do_print = true;  % Do or do not print out progress each iteration.

    % Helper lambda for getting the twist.
    get_twist = @(joint_angles) get_twist_ex(robot, joint_angles, Tsd);

    % Start with the initial guess.
    joint_angles = guess_joint_angles;

    iter = 0;
    twist_b = get_twist(joint_angles);
    iter_errang(iter+1) = norm(twist_b(1:3));
    iter_errlin(iter+1) = norm(twist_b(4:6));
    iter_cond(iter+1) = J_condition(J_body(robot, joint_angles));
    iter_stepnorm(iter+1) =  nan;
    if do_print
        fprintf('%4s  %8s  %9s  %10s  %12s  %s\n', 'Iter', 'StepNorm', 'ErrAng', 'ErrLin', 'Cond#', 'Joint Angles');    
        fprintf('%4d  %8f  %9f  %10f  %12.2f  %s\n', iter, iter_stepnorm(iter+1), iter_errang(iter+1), iter_errlin(iter+1), iter_cond(iter+1), mat2str(joint_angles', 5));
    end
    while norm(twist_b(1:3)) > angular_thresh || norm(twist_b(4:6)) > linear_thresh
        iter = iter + 1;
        Jb = J_body(robot, joint_angles);
        % Calculate partial derivative of manipulability wrt joints
        w0 = sqrt(det(Jb*Jb'));
        q0_dot = nan(1, 7);
        for i=1:7
            q0 = joint_angles;
            q0(i) = q0(i) + deltaq;
            Ji = J_body(robot,q0);
            wi= sqrt(det(Ji*Ji'));
            q0_dot(i) = (wi - w0) / deltaq;
        end
        % Calculate step
        J_daggr = dagger_J(Jb,7,6);
        step = J_daggr * twist_b * lr + (eye(7,7) - J_daggr *Jb) * k0 * q0_dot';
        % Update the solution
        joint_angles = mod(joint_angles + step, 2*pi);
        % Prepare state for next iteration
        twist_b = get_twist(joint_angles);
        % Save progress
        iter_errang(iter+1) = norm(twist_b(1:3));
        iter_errlin(iter+1) = norm(twist_b(4:6));
        iter_cond(iter+1) = J_condition(Jb);
        iter_stepnorm(iter+1) =  norm(step);
        % Print progress
        if do_print && mod(iter, 1) == 0
            fprintf('%4d  %8f  %9f  %10f  %12.2f  %s\n', iter, iter_stepnorm(iter+1), iter_errang(iter+1), iter_errlin(iter+1), iter_cond(iter+1), mat2str(joint_angles', 4));
        end
    end
end

function twist = get_twist_ex(robot, joint_angles, Tsd)
    trans = inv(FK_space(robot, joint_angles)) * Tsd;
    [screw, theta] = trans2screw(trans);
    twist = screw2twist(screw, theta);
end    



