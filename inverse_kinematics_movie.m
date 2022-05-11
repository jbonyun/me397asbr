function [joint_angles, iter_errang, iter_errlin, iter_cond, iter_step, iter_stepnorm] = inverse_kinematics_movie(robot, start_angles, dest_T, step_function, plot_title, plot_subtitle, video_fname, lr)
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
    if nargin < 6
        plot_subtitle = '';
    end

    % Some constants for the algorithm.
    angular_thresh = 999; %0.001;  % When to stop the search.
    linear_thresh = 1.0; %0.01;  % When to stop the search.
    joint_step_size_limit = pi/8;  % Max amount any joint can move in a step.
    iter_limit = 200;
    do_print = true;

    % Prepare the robot graphics model
    kuka = loadrobot('kukaIiwa14');

    % Prepare plot and make a lambda to simplify the call to update.
    plot_state = build_plot();
    sgtitle(sprintf("Inverse Kinematics via %s\n%s", plot_title, plot_subtitle));
    update_plot = @(jang, iter, linerr, angerr, maxjvel, J, cond, iso, cap_desc) make_plot(kuka, robot, plot_state, jang, dest_T, iter, linerr, angerr, maxjvel, J, cond, iso, cap_desc);

    % Helper lambda for getting the twist.
    get_twist = @(joint_angles) get_twist_ex(robot, joint_angles, dest_T);

    % Start with the initial guess.
    joint_angles = start_angles;

    % Iteration 0 calculations
    iter = 0;
    Jb = J_body(robot, joint_angles);
    twist_b = get_twist(joint_angles);
    iter_errang(iter+1) = norm(twist_b(1:3));
    iter_errlin(iter+1) = norm(twist_b(4:6));
    iter_cond(iter+1) = J_condition(Jb);
    iter_isotropy(iter+1) = J_isotropy(Jb);
    iter_step(iter+1, 1:robot.dof) = nan;
    iter_stepnorm(iter+1) = nan;
    update_plot(joint_angles, iter, iter_errlin, iter_errang, nan, Jb, iter_cond, iter_isotropy, '');
    frames(iter+1) = getframe(plot_state.f);
    if do_print
        fprintf('%4s  %8s  %9s  %10s  %12s  %s\n', 'Iter', 'StepNorm', 'ErrAng', 'ErrLin', 'Cond#', 'Joint Angles');    
        fprintf('%4d  %8f  %9f  %10f  %12.2f  %s\n', iter, iter_stepnorm(iter+1), iter_errang(iter+1), iter_errlin(iter+1), iter_cond(iter+1), mat2str(joint_angles', 5));
    end
    while (norm(twist_b(1:3)) > angular_thresh || norm(twist_b(4:6)) > linear_thresh) && iter < iter_limit
        drawnow;
        %pause(0.1);
        iter = iter + 1;
        % Calculate step
        step = step_function(joint_angles, trans2translation(dest_T)); %twist_b);
        % Limit the step size, scaling the whole step down if it's too
        % large in any joint.
        if max(abs(step)) > joint_step_size_limit
            step_scale = joint_step_size_limit / max([abs(step); joint_step_size_limit]);
            %capped_desc = sprintf('capped @ %.1f%%', step_scale * 100.);
            capped_desc = sprintf('BIG JVEL');
        else
            step_scale = 1.0;
            capped_desc = '';
        end
        %step = step * step_scale;  %% THIS IS PROBLEM
        % Update the solution
        joint_angles = mod(joint_angles + step, 2*pi);
        joint_angles = joint_angles - ((joint_angles>pi) * 2 * pi);
        % Prepare state for next iteration
        twist_b = get_twist(joint_angles);
        Jb = J_body(robot, joint_angles);
        % Save progress
        iter_errang(iter+1) = norm(twist_b(1:3));
        iter_errlin(iter+1) = norm(twist_b(4:6));
        iter_cond(iter+1) = J_condition(Jb);
        iter_isotropy(iter+1) = J_isotropy(Jb);
        iter_step(iter+1, 1:robot.dof) = step;
        iter_stepnorm(iter+1) =  norm(step);
        max_joint_vel = max(abs(step)) ./ lr;
        update_plot(joint_angles, iter, iter_errlin, iter_errang, max_joint_vel, Jb, iter_cond, iter_isotropy, capped_desc);
        frames(iter+1) = getframe(plot_state.f);
        % Print progress
        if do_print && mod(iter, 1) == 0
            fprintf('%4d  %8f  %9f  %10f  %12.2f  %s\n', iter, iter_stepnorm(iter+1), iter_errang(iter+1), iter_errlin(iter+1), iter_cond(iter+1), mat2str(joint_angles', 4));
        end
    end
    done_word = 'done';
    if iter >= iter_limit
        done_word = 'TIMEOUT';
    end
    max_joint_vel = max(max(abs(iter_step))) ./ lr;
    update_plot(joint_angles, iter, iter_errlin, iter_errang, max_joint_vel, Jb, iter_cond, iter_isotropy, capped_desc);
    sgtitle(sprintf("Inverse Kinematics via %s\n%s (%s)", plot_title, plot_subtitle, done_word));
    frames(iter+2) = getframe(plot_state.f);
    % Make video file
    writer = VideoWriter(video_fname, "MPEG-4");
    lr_per_second = 0.5;
    frames_per_second = min(30, max(floor(lr_per_second / lr), 1));
    writer.FrameRate = frames_per_second;
    fprintf('Saving video file to %s at %f fps\n', video_fname, frames_per_second);
    open(writer);
    for i = 1:numel(frames)
        frames(i).cdata = imresize(frames(i).cdata, 0.5);
        writeVideo(writer, frames(i));
    end
    close(writer);
    fprintf('Video saved\n');
end

function twist = get_twist_ex(robot, joint_angles, Tsd)
    trans = inv(FK_space(robot, joint_angles)) * Tsd;
    [screw, theta] = trans2screw(trans);
    twist = screw2twist(screw, theta);
end

function [st] = build_plot()
    iternum = 0;
    linerr = nan;
    angerr = nan;
    st.f = figure;
    fpos = st.f.get('Position');
    %st.f.set('Position', [fpos(1) fpos(2) fpos(3) fpos(3) * 2]);
    %st.f.set('Position', [fpos(1) fpos(2) 1300 600]);  % x, y (from bottom), width, height
    st.f.set('Position', [50 50 1300 600]);  % x, y (from bottom), width, height
    %st.f.set('Position', [50 1200 1300 600]);  % x, y (from bottom), width, height
    definesubplot = @(loc) subtightplot(2, 4, loc, [0.065 0.065], [0.10 0.16], [0.03 0.04]);
    st.ax_pic = definesubplot([1 2 5 6]);
    st.ax_err = definesubplot(3);
    st.ax_condiso = definesubplot(4);
    st.ax_ellipse_lin = definesubplot(7);
    st.ax_ellipse_ang = definesubplot(8);
    st.text_iter = annotation('textbox', [0.01 .95 0.01 0.01], 'String', sprintf('Iter %2d', iternum), 'FitBoxToText', true, 'LineStyle', 'none', 'FontWeight', 'bold', 'FontName', 'Times');
    st.text_err = annotation('textbox', [0.01 .90 0.01 0.01], 'String', sprintf('Error\nLin: %.4f\nAng: %.4f\n\nJVel: nan', linerr, angerr), 'FitBoxToText', true, 'LineStyle', 'none', 'FontWeight', 'bold', 'FontName', 'Times');
    st.text_capped = annotation('textbox', [0.01 .70 0.01 0.01], 'String', sprintf('uncapped'), 'FitBoxToText', true, 'LineStyle', 'none', 'FontWeight', 'bold', 'FontName', 'Times');
end

function [ax] = make_plot(model, robot, state, joint_angles, dest_T, iternum, linerr, angerr, max_joint_vel, J, condnum, isotropy, capped_desc)
    subplot(state.ax_pic);
    ax = show(model, getrobotconfig(model, joint_angles), 'Frames', 'off'); %, 'FastUpdate', true, 'PreservePlot', false);
    subplot(state.ax_pic);
    xlim([-0.9 0.9]);
    ylim([-0.9 0.9]);
    zlim([-0.1 1.5]);
    view([-195 15]);
    hold all;
    plot_3d_axis_transform(FK_space(robot, joint_angles), 'ax', ax, 'originscale', 0.001, 'scale', 0.2, 'LineWidth', 1);
    plot_3d_axis_transform(dest_T, 'ax', ax, 'originscale', 0.001, 'scale', 0.1, 'LineWidth', 3);
    state.text_iter.String = sprintf('Iter %2d', iternum);
    state.text_err.String = sprintf('Error\nLin: %.3f\nAng: %.3f\n\nJVel: %.1f', linerr(end), angerr(end), max_joint_vel);
    state.text_capped.String = capped_desc;
    hold off;
    subplot(state.ax_ellipse_lin);
    ellipsoid_plot_linear(J, 'Axis', gca);
    view([-195 15]);
    ylim([-2 2]); xlim([-2 2]); ylim([-2 2]);
    subplot(state.ax_ellipse_ang);
    ellipsoid_plot_angular(J, 'Axis', gca);
    view([-195 15]);
    subplot(state.ax_err);
    yyaxis left;
    semilogy(linerr);
    ylim([1e-1 inf]);
    ylabel('Linear');
    yyaxis right;
    semilogy(angerr);
    ylim([1e-2 inf]);
    ylabel('Angular');
    title('Log Error');
    subplot(state.ax_condiso);
    yyaxis left;
    plot(condnum);
    ylim([0 inf]);
    ylabel('Cond#');
    %hold on;
    yyaxis right;
    plot(isotropy);
    ylim([0 inf]);
    %semilogy(isotropy);
    ylabel('Isotropy');
    title('Condition/Isotropy');
end
