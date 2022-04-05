function joint_angle=J_inverse_kinematics(robot,config,Tsd)
    %W8-L2
    %input robot configuartion A and desired tranformation matrix
    %output joint_angles

    i=0;
    %joint_angle=[0.2,0.2,0.2,0.2,0.2,0.2,0.2];
    joint_angle=config;
    joint_angle=joint_angle';
    [skew_b,angle]=logmatirxs(inv(FK_space(robot,joint_angle))*Tsd);
    angle=vpa(angle,4);
    twist_b = skew_b*angle;
    while norm(twist_b(1:3))>0.05 || norm(twist_b(4:6))>0.05
        Jb=J_body(robot, joint_angle);
        J_daggr=dagger_J(Jb,7,6);
        joint_angle=joint_angle+ J_daggr*twist_b*0.1;   
        i=i+1;
        [skew_b,angle]=logmatirxs(inv(FK_space(robot,joint_angle))*Tsd);
        angle=vpa(angle,4);
        twist_b = skew_b*angle; 
    end
