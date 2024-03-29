function end_frame = FK_space(robot, joint_angles, varargin)
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
    % Source: Alambeigi, F. ASBR Lecture Notes. 2022, W6-L1, p. 10.

    p = inputParser;
    addRequired(p, 'robot'); 
    addRequired(p, 'joint_angles'); 
    addParameter(p, 'DoPlot', false);
    addParameter(p, 'JointNum', nan);
    parse(p, robot, joint_angles, varargin{:});
    args = p.Results;

    if isnan(args.JointNum)
        maxn = robot.dof;
    else
        maxn = args.JointNum;
    end

    if args.DoPlot
        axis_scale = max(max(robot.offset)) / 20;
        figure;
        axis equal;
        view(135, 7);
        xlabel('x'); ylabel('y'); zlabel('z');
        plot_3d_axis([0 0 0]', [1 0 0]', [0 1 0]', [0 0 1]','scale', axis_scale*2);
        hold on;
    end

    cumulative_transform = [eye(3) [0; 0; 0]; 0 0 0 1];
    for i = 1:maxn
        skew_s = skewsym(robot.screw(:, i));
        cumulative_transform = cumulative_transform * expm(skew_s * joint_angles(i));
        if args.DoPlot
            joint_home = [eye(3) robot.offset(i,:)'; 0 0 0 1];
            joint_frame = cumulative_transform * joint_home;
            plot_3d_axis_transform(joint_frame, 'scale', axis_scale);
            plot_joint_axis(joint_frame, robot.axes(i,:)', axis_scale*2);
        end
    end

    if isnan(args.JointNum)
        end_frame = cumulative_transform * robot.home;
    else
        joint_home = [eye(3) robot.offset(args.JointNum,:)'; 0 0 0 1];
        end_frame = cumulative_transform * joint_home;
    end
    
    if args.DoPlot
        plot_3d_axis_transform(end_frame, 'ax', gca, 'scale', axis_scale*2);
        hold off;
    end
