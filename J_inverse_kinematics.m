function J_inverse_kinematics(robot,Tsd)
    
    i=0;
    joint_angle=[10,10,10,10,10,10,10];
    skew_b=logm(inv(FK_body(robot,joint_angle))*Tsd);
    twist_b = unskewsym(skew_b);
    Jb=J_body(robot, joint_angle);
    J_daggr=dagger_J(Jb,7,6);
    joint_angle=joint_angle+ J_daggr*twist_b;
    i=i+1;
    while norm(twist_b(1:3))>0.5 || norm(twist_b(4:6))>0.5
                
        skew_b=logm(FK_body(robot,joint_angle)*Tsd);
        twist_b = unskewsym(skew_b);
        Jb=J_body(robot, joint_angle);
        J_daggr=dagger_J(Jb,7,6);
        joint_angle=joint_angle+ J_daggr*twist_b;
        i=i+1;
    end
