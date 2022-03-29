function end_frame = FK_body(robot, joint_angles, varargin)
    % Calculates the forward kinematics in the space frame.
    % Inputs:
    %   robot: struct with robot description
    %            field dof: integer number of degrees of freedom
    %            field home: 4x4 homogeneous matrix defining home position
    %            field bscrew: 6xdof matrix of body screw vectors for each joint
    %   joint_angles: angles of each joint, in rad
    % Outputs:
    %   end_frame: 4x4 transformation matrix from the origin of
    %              the body frame to the origin of the space frame.
    % Jeff Bonyun (jb79332), jbonyun@utexas.edu, 20220322
    % On behalf of the Sun/Bonyun team for ME397 ASBR, Spring 2022.
    % Source: [none yet]

    p = inputParser;
    addRequired(p, 'robot'); 
    addRequired(p, 'joint_angles'); 
    addParameter(p, 'DoPlot', false);
    parse(p, robot, joint_angles, varargin{:});
    args = p.Results;

    if args.DoPlot
        axis_scale = max(max(robot.offset)) / 20;
        figure;
        axis equal;
        view(135, 7);
        xlabel('x'); ylabel('y'); zlabel('z');
        plot_3d_axis([0 0 0]', [1 0 0]', [0 1 0]', [0 0 1]', 'scale', axis_scale*2);
        hold on;
    end

%    cumulative_transform = [eye(3) [0; 0; 0]; 0 0 0 1];
%     for i = 1:robot.dof
%         b_screw = robot.bscrew(:, i);
%         skew_b_screw = skewsym(b_screw);
%         cumulative_transform = cumulative_transform * expm(skew_b_screw * joint_angles(i));
%         if args.DoPlot
%             joint_home = [eye(3) robot.offset(i,:)'; 0 0 0 1];
%             joint_frame = joint_home * cumulative_transform;
%             plot_3d_axis_transform(joint_frame, 'scale', axis_scale);
%             plot_joint_axis(joint_frame, robot.axes(i,:)', axis_scale*2);
%         end
%     end
    cumulative_transform = [eye(3) [0; 0; 0]; 0 0 0 1];
    for i = 1:robot.dof       
         b_screw = robot.bscrew(:, i);
         skew_b_screw = skewsym(b_screw);
         cumulative_transform = cumulative_transform * expm(skew_b_screw * joint_angles(i));
         % Which way?
         % should we count down from dof to 1 too?
         % cumulative_transform = expm(skew_b_screw * joint_angles(i)) * cumulative_transform;
    end
    end_frame = robot.home * cumulative_transform;

    if args.DoPlot
        for i = 1:robot.dof
            cumulative_transform = [eye(3) [0; 0; 0]; 0 0 0 1];
            for j = i:robot.dof
                b_screw = robot.bscrew(:, j);
                skew_b_screw = skewsym(b_screw);
                cumulative_transform = cumulative_transform * expm(skew_b_screw * joint_angles(j));
            end
            joint_home = [eye(3) robot.offset(i,:)'; 0 0 0 1];
            joint_frame = joint_home * cumulative_transform;
            plot_3d_axis_transform(joint_frame, 'scale', axis_scale);
            plot_joint_axis(joint_frame, robot.axes(i,:)', axis_scale*2);
        end
        plot_3d_axis_transform(end_frame, 'ax', gca, 'scale', axis_scale*2);
        hold off;
    end