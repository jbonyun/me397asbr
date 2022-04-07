function end_frame = FK_space_sym(robot, joint_angles)
    % Calculates the forward kinematics in the space frame.
    % Inputs:
    %   robot: struct with robot description
    %            field home: 4x4 homogeneous matrix defining home position
    %            field screw: 6xn matrix of screw vectors for each joint
    %   joint_angles: angles of each joint, in rad
    % Outputs:
    %   end_frame: 4x4 transformation matrix from the origin of
    %              the space frame to the final location and orientation.
    % W6-l2 page 4

    %n=size(joint_angles,2);
    cumulative_transform = expm_sym(robot.screw(:, 1), joint_angles(1));
    for i = 2:7
        cumulative_transform = cumulative_transform * expm_sym(robot.screw(:, i), joint_angles(i));

    end

    end_frame = cumulative_transform * robot.home;
