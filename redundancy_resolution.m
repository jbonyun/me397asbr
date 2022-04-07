function joint_angle=redundancy_resolution(robot,config,Tsd)
    
%input robot configuartion A and desired tranformation matrix
%output joint_angles

    joint_angle=config;
    joint_angle=joint_angle';
    i=0;
    [skew_b,angle]=logmatirxs(inv(FK_space_sym(robot,joint_angle))*Tsd);
    angle=vpa(angle,4);
    twist_b = skew_b*angle;
    deltq=0.1;
    Jb=J_body(robot, joint_angle);
    w0=sqrt(det(Jb*Jb'));
    
    while norm(twist_b(1:3))>0.05 || norm(twist_b(4:6))>0.05
        for i=1:7
            q0=joint_angle;
            q0(i)=q0(i)+deltq;
            Ji=J_body(robot,q0);
            wi=sqrt(det(Ji*Ji'));
            q0_dot(i)=(wi-w0)/deltq;
            q0_dot(i)=vpa(q0_dot(i),4)*1e-7;
        end
        q0_dot;
        Jb=J_body(robot, joint_angle);
        J_daggr=dagger_J(Jb,7,6);
        joint_angle=joint_angle+J_daggr*twist_b*0.1+(eye(7,7)-J_daggr*Jb)*q0_dot';
        %joint_angle=joint_angle+J_daggr*twist_b*0.1;
        i=i+1;
        [skew_b,angle]=logmatirxs(inv(FK_space_sym(robot,joint_angle))*Tsd);
        angle=vpa(angle,4);
        twist_b = skew_b*angle 
    end