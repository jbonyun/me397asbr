function end_frame = FK_space(robot, joint_angles)
    % Calculates the forward kinematics in the space frame.
    % Inputs:
    %   robot: struct with robot description
    %            field home: 4x4 homogeneous matrix defining home position
    %            field screw: 6xn matrix of screw vectors for each joint
    %   joint_angles: angles of each joint, in rad
    % Outputs:
    %   end_frame: 4x4 transformation matrix from the origin of
    %              the space frame to the final location and orientation.
    % Jeff Bonyun (jb79332), jbonyun@utexas.edu, 20220322
    % On behalf of the Sun/Bonyun team for ME397 ASBR, Spring 2022.
    % Source: [none yet]

    do_plot = true;

    if ndims(joint_angles) == 1
        n = numel(joint_angles);
    else
        n = size(joint_angles,1);
    end

    if do_plot
        axis_scale = max(max(robot.offset)) / 50;
        figure;
        axis equal;
        plot_3d_axis([0 0 0]', [1 0 0]', [0 1 0]', [0 0 1]','scale', axis_scale*2);
        hold on;
    end

    end_frame = [eye(3) [0; 0; 0]; 0 0 0 1];
    for i = 1:n
        skew_s = skewsym(robot.screw(:, i));
        end_frame = end_frame * expm(skew_s * joint_angles(i));
        if do_plot
            joint_home = [eye(3) robot.offset(i,:)'; 0 0 0 1];
            joint_frame = end_frame * joint_home;
            plot_3d_axis_transform(joint_frame, 'scale', axis_scale);
            plot_joint_axis(joint_frame, robot.axes(i,:)', axis_scale*2);
        end
    end

    end_frame = end_frame * robot.home;
    
    if do_plot
        plot_3d_axis_transform(end_frame, 'ax', gca, 'scale', axis_scale*2);
        hold off;
    end