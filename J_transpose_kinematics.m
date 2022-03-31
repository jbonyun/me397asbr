function J_transpose_kinematics(robot,Xd)
	


    Xe=[0;0;0;0;0;0];
    q=[0;0;0;0;0;0];
    err=Xd-Xe;
    V = diag([1 2 3]);
    U =[1 0 0;0 1 0;0 0 1];
    K=U*V*U';
    while norm(err)>0.5
        q_dot=transpose(J_space(robot,q))*K*err;
        q=q+q_dot;
        transform=FK_space(robot,q);
        Xe=trans2vector(transform);
        err=Xd-Xe;
    end
    



