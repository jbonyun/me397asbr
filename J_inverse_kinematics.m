function joint_angle=J_inverse_kinematics(robot,Tsd)
    %W8-L2
    %input robot and configuartion (Tsd)
    %output joint_angles
    




    i=0;
    %joint_angle=[0.2,0.2,0.2,0.2,0.2,0.2,0.2];
    joint_angle=[0.3,0.3,0.3,0.3,0.3,0.3,0.3];
    [skew_b,angle]=logmatirxs(inv(FK_space(robot,joint_angle))*Tsd);
    angle=vpa(angle,4);
    twist_b = skew_b*angle;
    while norm(twist_b(1:3))>0.05 || norm(twist_b(4:6))>0.05
        Jb=J_body(robot, joint_angle);
        J_daggr=dagger_J(Jb,7,6);
        joint_angle=joint_angle+ J_daggr*twist_b   
        i=i+1;
        [skew_b,angle]=logmatirxs(inv(FK_space(robot,joint_angle))*Tsd);
        angle=vpa(angle,4);
        twist_b = skew_b*angle ; 
    end
