function q=J_transpose_kinematics(robot,config,Xd)
	%W10-L1  page 15
    %input: robot , initial config A and desired Xd
    %output: joint angles of desired Xd
    q=config;
    Xd=vpa(Xd,4);
    transform=FK_space(robot,q);
    Xe=trans2vector(transform);
    Xe=vpa(Xe,4);
    err=Xd-Xe;
    V = diag([1 1 1 2 2 2]);
    %U =[1 1 1 1 1 1;1 1 1 1 1 1;1 1 1 1 1 1;1 1 1 1 1 1;1 1 1 1 1 1; 1 1 1 1 1 1];
    U =[1 0 0 0 0 0;0 1 0 0 0 0 ;0 0 1 0 0 0;0 0 0 1 0 0;0 0 0 0 1 0;0 0 0 0 0 1];
    K=U*V*U';

    while norm(err)>1
        Jacobian=J_body_sym(robot,q);
        q_dot=transpose(Jacobian)*K*err;
        q=q+q_dot*0.0000001
        transform=FK_space_sym(robot,q);
        Xe=trans2vector(transform);
        Xe=vpa(Xe,4);
        err=Xd-Xe;
        norm(err);        
    end



