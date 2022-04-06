function end_frame = FK_body(robot, joint_angles)
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
    % Source: Alambeigi, F. ASBR Lecture Notes. 2022, W6-L2, p. 6.

    % Calculate the full transform to the end effector.
    cumulative_transform = [eye(3) [0; 0; 0]; 0 0 0 1];
    for i = 1:robot.dof
         b_screw = robot.bscrew(:, i);
         skew_b_screw = skewsym(b_screw);
         cumulative_transform = cumulative_transform * expm(skew_b_screw * joint_angles(i));
    end
    end_frame = robot.home * cumulative_transform;
