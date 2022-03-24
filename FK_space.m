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

    if ndims(joint_angles) == 1
        n = numel(joint_angles);
    else
        n = size(joint_angles,1);
    end
    end_frame = expm(skewsym(robot.screw(:, 1)) * joint_angles(1));
    for i = 2:n
        skew_s = skewsym(robot.screw(:, i));
        end_frame = end_frame * expm(skew_s * joint_angles(i));
    end
    end_frame = end_frame * robot.home;