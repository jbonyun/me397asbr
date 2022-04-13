function [joint_angles, iter_errang, iter_errlin, iter_cond, iter_stepnorm] = inverse_kinematics_movie(robot, start_angles, dest_T, step_function)
    % Plot the iterations of J-inverse IK
    % Inputs:
    %   robot: struct with robot description
    %   start_angles: nx1 starting guess for joint angles.
    %   dest_T: 4x4 desired configuration, as a transformation.
    % Output:
    %    joint_angles: nx1 joint angles in radians that will acheive the
    %                  requested transformation.
    %    iter_*: kx1 iteration-by-iteration progress of metrics.
    % On behalf of the Sun/Bonyun team for ME397 ASBR, Spring 2022.
    % Source: Alambeigi, F. ASBR Lecture Notes. 2022, W10-L1 p. 9.

    % Suppress annoying warning about inv
    %#ok<*MINV> 

    % Some constants for the algorithm.
    angular_thresh = 0.001;  % When to stop the search.
    linear_thresh = 0.01;  % When to stop the search.
    do_print = true;

    % Prepare the robot graphics model
    kuka = loadrobot('kukaIiwa14');

    update_plot = @(jang) make_plot(kuka, robot, jang, dest_T);
    figure;

    % Plot the starting and ending transforms
%     config = getrobotconfig(kuka, start_angles);
%     figure;
%     ax = show(kuka, config, 'Frames', 'off');
%     xlim([-0.9 0.9]);
%     ylim([-0.9 0.9]);
%     zlim([-0.1 1.5]);
%     view([-195 15]);
%     hold all;
%     plot_3d_axis_transform(FK_space(robot, start_angles), 'ax', ax, 'originscale', 0.001, 'scale', 0.2, 'LineWidth', 1);
%     plot_3d_axis_transform(dest_T, 'ax', ax, 'originscale', 0.001, 'scale', 0.1, 'LineWidth', 3);
%     hold off;
    update_plot(start_angles);

    % Helper lambda for getting the twist.
    get_twist = @(joint_angles) get_twist_ex(robot, joint_angles, dest_T);

    % Start with the initial guess.
    joint_angles = start_angles;

    iter = 0;
    twist_b = get_twist(joint_angles);
    iter_errang(iter+1) = norm(twist_b(1:3));
    iter_errlin(iter+1) = norm(twist_b(4:6));
    iter_cond(iter+1) = J_condition(J_body(robot, joint_angles));
    iter_stepnorm(iter+1) = nan;
    if do_print
        fprintf('%4s  %8s  %9s  %10s  %12s  %s\n', 'Iter', 'StepNorm', 'ErrAng', 'ErrLin', 'Cond#', 'Joint Angles');    
        fprintf('%4d  %8f  %9f  %10f  %12.2f  %s\n', iter, iter_stepnorm(iter+1), iter_errang(iter+1), iter_errlin(iter+1), iter_cond(iter+1), mat2str(joint_angles', 5));
    end
    while norm(twist_b(1:3)) > angular_thresh || norm(twist_b(4:6)) > linear_thresh
        pause(0.5);
        iter = iter + 1;
        % Calculate step
        step = step_function(joint_angles, twist_b);
        % Update the solution
        joint_angles = mod(joint_angles + step, 2*pi);
        joint_angles = mod(joint_angles + step, 2*pi);
        % Prepare state for next iteration
        twist_b = get_twist(joint_angles);
        % Save progress
        iter_errang(iter+1) = norm(twist_b(1:3));
        iter_errlin(iter+1) = norm(twist_b(4:6));
        Jb = J_body(robot, joint_angles);
        iter_cond(iter+1) = J_condition(Jb);
        iter_stepnorm(iter+1) =  norm(step);
        update_plot(joint_angles);
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

function [ax] = make_plot(model, robot, joint_angles, dest_T)
    ax = show(model, getrobotconfig(model, joint_angles), 'Frames', 'off'); %, 'FastUpdate', true, 'PreservePlot', false);
    xlim([-0.9 0.9]);
    ylim([-0.9 0.9]);
    zlim([-0.1 1.5]);
    view([-195 15]);
    hold all;
    plot_3d_axis_transform(FK_space(robot, joint_angles), 'ax', ax, 'originscale', 0.001, 'scale', 0.2, 'LineWidth', 1);
    plot_3d_axis_transform(dest_T, 'ax', ax, 'originscale', 0.001, 'scale', 0.1, 'LineWidth', 3);
    hold off;
end
