clear;
syms x1 x2 x3 x4 x5 x6 x7
syms d1 d2 d3 d4 d5 real

robot.home = [eye(3,3) [0; 0; 1180]; 0 0 0 1];
    
    % In home position, the direction of the axis of rotation for each joint.
    % Positive rotations are CCW when looking in the given direction.
    robot.axes = [
        0  0  1;
        0  1  0;
        0  0  1;
        0 -1  0;
        0  0  1;
        0  1  0;
        0  0  1;
        ];
    % In home position, the location of the center of each joint, in mm, in
    % world coordinates (center of base on table).

%     robot.offset = [
%         0 0 0;
%         0 0 d1;
%         0 0 0;
%         0 0 d1+d2+d3;
%         0 0 0;
%         0 0 d1+d2+d3+d4+d5;
%         0 0 0;
%         ];

    robot.offset = [
        0 0 0;
        0 0 360;
        0 0 0;
        0 0 780;
        0 0 0;
        0 0 1180;
        0 0 0;
        ];
    % Extract DOF from the parameters we have already given.
    robot.dof = size(robot.axes,1);
    
    % Calculate the screw vectors from each joint's axis at home and translation
    % at home.
    for i = 1:robot.dof
        v = -cross(robot.axes(i,:), robot.offset(i,:));
        robot.screw(:, i) = [robot.axes(i,:)'; v'];
    end
    for i = 1:robot.dof
        robot.bscrew(:, i) = adjoint_transform(inv(robot.home)) * robot.screw(:, i);
    end

%joint_angles=[0.1;0.1;0.1;0.1;0.1;0.1;0.1];
%joint_angles=[0;0;0;0;0;0;0];
%joint_angles=[10;0;10;10;10;0;10];
%joint_angles=[1,1,1,1,1,1,1];
joint_angles=[0.2;0.2;0.2;0.2;0.2;0.2;0.2];
%joint_angles=[0.4;0.4;0.4;0.6;0.9;0.4;0.4];
J = sym('a%d%d',[6,7]);
J(:, 1) = robot.screw(:, 1);
prod_expon = expm_sym(robot.screw(:, 1) , joint_angles(1));
for i = 2:robot.dof
    ad=adjoint_sym(prod_expon);
    J(:, i) = ad * robot.screw(:, i);
    prod_expon = prod_expon * expm_sym(robot.screw(:, i) , joint_angles(i));
end

end_fram=FK_space_sym(robot,joint_angles)

 %%
 
    %q=[0.19;0.19;0.19;0.19;0.19;0.19;0.19];
    q=[0.1;0.1;0.1;0.1;0.1;0.1;0.1];
    Xd=trans2vector(end_fram);
    Xd=vpa(Xd,6)
    transform=FK_body(robot,q);
    Xe=trans2vector(transform);
    Xe=vpa(Xe,6)
    err=Xd-Xe;
    V = diag([10 10 10 1 1 1]);
    %U =[1 1 1 1 1 1;1 1 1 1 1 1;1 1 1 1 1 1;1 1 1 1 1 1;1 1 1 1 1 1; 1 1 1 1 1 1];
    %U =[1 0 0 0 0 0;0 1 0 0 0 0 ;0 0 1 0 0 0;0 0 0 1 0 0;0 0 0 0 1 0;0 0 0 0 0 1];
    U = orth(eye(6));
    K=U*V*U';
    time_step=0.000001;


    %A=pascal(6);
    Error=(norm(err));
    i=1;
    while norm(err)>3
        Jacobian=J_space(robot,q);
        q_dot=transpose(Jacobian)*K*err;
        q=q+q_dot*time_step;
        transform=FK_body(robot,q);
        Xe=trans2vector(transform);
        Xe=vpa(Xe,6);
        %Xe=Xe+Jacobian*q_dot*time_step;
        err=Xd-Xe;
        V_dot=-err'*K*Jacobian*q_dot;
        norm(err);
        norm2=err'*K*err
        Error(end+1)=norm(err);
        i=i+1;
    end
I=1:i;
plot(I,Error,'-o');
xlabel('Number of iteration') ;
ylabel('Norm of error');
title('J transpose kinematics');

%%
   q=[0.1;0.1;0.1;0.1;0.1;0;1];
   q_dot=q;
   Jacobian=J_space(robot,q);
    transform=FK_space(robot,q);
    Xe=trans2vector(transform);
    Xe=vpa(Xe,6);
    Xe_dot=Jacobian*q_dot
    transform=FK_space(robot,q+q_dot);
    Xe_2=trans2vector(transform);
    Xe_2=vpa(Xe_2,6);
    Xe_2-Xe