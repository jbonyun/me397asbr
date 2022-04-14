function [joint_angles, iter_errang, iter_errlin, iter_cond, iter_stepnorm] = inverse_kinematics_movie(robot, start_angles, dest_T, step_function, plot_title)
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
    do_print = true;

    % Prepare the robot graphics model
    kuka = loadrobot('kukaIiwa14');

    % Prepare plot and make a lambda to simplify the call to update.
    plot_state = build_plot();
    sgtitle(sprintf("%s Inverse Kinematics\n ", plot_title));
    update_plot = @(jang, iter, linerr, angerr, J, cond, iso) make_plot(kuka, robot, plot_state, jang, dest_T, iter, linerr, angerr, J, cond, iso);

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
    iter_stepnorm(iter+1) = nan;
    update_plot(joint_angles, iter, iter_errlin, iter_errang, Jb, iter_cond, iter_isotropy);
    frames(iter+1) = getframe(plot_state.f);
    if do_print
        fprintf('%4s  %8s  %9s  %10s  %12s  %s\n', 'Iter', 'StepNorm', 'ErrAng', 'ErrLin', 'Cond#', 'Joint Angles');    
        fprintf('%4d  %8f  %9f  %10f  %12.2f  %s\n', iter, iter_stepnorm(iter+1), iter_errang(iter+1), iter_errlin(iter+1), iter_cond(iter+1), mat2str(joint_angles', 5));
    end
    while norm(twist_b(1:3)) > angular_thresh || norm(twist_b(4:6)) > linear_thresh
        drawnow;
        %pause(0.1);
        iter = iter + 1;
        % Calculate step
        step = step_function(joint_angles, twist_b);
        % Update the solution
        joint_angles = mod(joint_angles + step, 2*pi);
        % Prepare state for next iteration
        twist_b = get_twist(joint_angles);
        Jb = J_body(robot, joint_angles);
        % Save progress
        iter_errang(iter+1) = norm(twist_b(1:3));
        iter_errlin(iter+1) = norm(twist_b(4:6));
        iter_cond(iter+1) = J_condition(Jb);
        iter_isotropy(iter+1) = J_isotropy(Jb);
        iter_stepnorm(iter+1) =  norm(step);
        update_plot(joint_angles, iter, iter_errlin, iter_errang, Jb, iter_cond, iter_isotropy);
        frames(iter+1) = getframe(plot_state.f);
        % Print progress
        if do_print && mod(iter, 1) == 0
            fprintf('%4d  %8f  %9f  %10f  %12.2f  %s\n', iter, iter_stepnorm(iter+1), iter_errang(iter+1), iter_errlin(iter+1), iter_cond(iter+1), mat2str(joint_angles', 4));
        end
    end
    sgtitle(sprintf("%s Inverse Kinematics\n(done)", plot_title));
    % Make video file
    video_fname = sprintf('video_%s_throughZeroPos_lr0.01.mp4', plot_title);
    fprintf('Saving video file to %s\n', video_fname);
    writer = VideoWriter(video_fname, "MPEG-4");
    writer.FrameRate = 20;
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
    %fpos = st.f.get('Position');
    %st.f.set('Position', [fpos(1) fpos(2) fpos(3) fpos(3) * 2]);
    st.f.set('Position', [50 1000 700 1100]);  % x, y (from bottom), width, height
    st.ax_pic = subplot(4, 2, [1 2 3 4]);
    st.ax_err = subplot(4, 2, 5);
    st.ax_condiso = subplot(4, 2, 6);
    st.ax_ellipse_lin = subplot(4, 2, 7);
    st.ax_ellipse_ang = subplot(4, 2, 8);
    st.text_iter = annotation('textbox', [0.01 .95 0.01 0.01], 'String', sprintf('Iter %2d', iternum), 'FitBoxToText', true, 'LineStyle', 'none', 'FontWeight', 'bold', 'FontName', 'Times');
    st.text_err = annotation('textbox', [0.82 .97 0.01 0.01], 'String', sprintf('Error\nLin: %.4f\nAng: %.4f', linerr, angerr), 'FitBoxToText', true, 'LineStyle', 'none', 'FontWeight', 'bold', 'FontName', 'Times');
end

function [ax] = make_plot(model, robot, state, joint_angles, dest_T, iternum, linerr, angerr, J, condnum, isotropy)
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
    state.text_err.String = sprintf('Error\nLin: %.3f\nAng: %.3f', linerr(end), angerr(end));
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
    ylim([0.01 inf]);
    ylabel('Linear');
    yyaxis right;
    semilogy(angerr);
    ylim([0.001 inf]);
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
