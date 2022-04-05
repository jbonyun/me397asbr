function J_transpose_kinematics(robot,Xd)
	%W10-L1  page 15
    %Xe=[0;0;0;0;0;0];
    q=[0.1;0.1;0.1;0.1;0.1;0.1];
    transform=FK_space(robot,q);
    Xe=trans2vector(transform);
    err=Xd-Xe;
    V = diag([1 2 3]);
    U =[1 0 0;0 2 0;0 0 3];
    K=U*V*U';
    while norm(err)>0.05
        q_dot=transpose(J_space_sym(robot,q))*K*err;
        q=q+q_dot;
        transform=FK_space_sym(robot,q);
        Xe=trans2vector(transform);
        err=Xd-Xe;
    end
    



