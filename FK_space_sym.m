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


    n = size(joint_angles);
    cumulative_transform = [eye(3) [0; 0; 0]; 0 0 0 1];
    for i = 1:n
        cumulative_transform = cumulative_transform * expm_sym(robot.screw(:, i), joint_angles(i));

    end

    end_frame = cumulative_transform * robot.home;
