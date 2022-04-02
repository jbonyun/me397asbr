function end_frame = FK_body_sym(robot, joint_angles)
 

    end_frame = robot.home;
    for i = 1:robot.dof
        
        b_screw = robot.bscrew(:, i);
 
        end_frame = end_frame * expm_sym(b_screw , joint_angles(i));
    end