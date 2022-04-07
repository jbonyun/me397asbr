clear;
syms x1 x2 x3 x4 x5 x6 x7 real
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
    robot.dof = 7;
    
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
%joint_angles=[0.2;0.2;0.2;0.2;0.2;0.2;0.2];
%joint_angles=[0.4;0.4;0.4;0.4;0.4;0.4;0.4];
%joint_angles=[x1,x2,x3,x4,x5,x6,x7];
joint_angles=[0.1;0.1;0.1;0.1;0.1;0.1;0.1];
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
w0=sqrt(det(J*J'));
q1=joint_angles;
q1(1)=q1(1)+0.1;
J_1=J_body(robot,q1);
w1=sqrt(det(J_1*J_1'));
q0_dot(1)= (w1-w0)/0.1;
q0_dot(1)=vpa(q0_dot(1),4)
%%
deltq=0.1;
w0=sqrt(det(J*J'));
for i=1:7
    q0=joint_angles;
    q0(i)=q0(i)+deltq;
    Ji=J_body(robot,q0);
    wi=sqrt(det(Ji*Ji'));
    q0_dot(i)=(wi-w0)/deltq;
    q0_dot(i)=vpa(q0_dot(i),4);
end
q0_dot




%%

   %joint_angle=[0.2,0.2,0.2,0.2,0.2,0.2,0.2];
    %joint_angle=[0.3,0.3,0.3,0.3,0.3,0.3,0.3];
    joint_angle=[-0.1,-0.1,-0.1,-0.1,-0.1,-0.1,-0.1];
    joint_angle=joint_angle';
    i=0;
    [skew_b,angle]=logmatirxs(inv(FK_space_sym(robot,joint_angle))*end_fram);
    angle=vpa(angle,4);
    twist_b = skew_b*angle;
    deltq=0.1;
    w0=sqrt(det(J*J'));
    
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
        [skew_b,angle]=logmatirxs(inv(FK_space_sym(robot,joint_angle))*end_fram);
        angle=vpa(angle,4);
        twist_b = skew_b*angle 
    end
%%

kuka = importrobot('iiwa14.urdf');
config = homeConfiguration(kuka);

    joint_angle=[-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3];
        for i=1:7
            config(i).JointPosition = double(joint_angle(i));
        end
    %show(kuka,config)
    joint_angle=joint_angle';
    q=0;
    [skew_b,angle]=logmatirxs(inv(FK_space_sym(robot,joint_angle))*end_fram);
    angle=vpa(angle,4);
    twist_b = skew_b*angle;
    deltq=0.1;
    w0=sqrt(det(J*J'));
    W=[w0];
    Twist_r=[norm(twist_b(1:3))];
    Twist_t=[norm(twist_b(4:6))];
    while norm(twist_b(1:3))>0.05 || norm(twist_b(4:6))>0.05
        for i=1:7
            q0=joint_angle;
            q0(i)=q0(i)+deltq;
            Ji=J_body(robot,q0);
            wi=sqrt(det(Ji*Ji'));
            q0_dot(i)=(wi-w0)/deltq;
            q0_dot(i)=vpa(q0_dot(i),4)*1e-7;
        end
        W(end+1)=wi;
        q0_dot;
        Jb=J_body(robot, joint_angle);
        J_daggr=dagger_J(Jb,7,6);
        joint_angle=joint_angle+J_daggr*twist_b*0.1+(eye(7,7)-J_daggr*Jb)*q0_dot'
        %joint_angle=joint_angle+J_daggr*twist_b*0.1;
        q=q+1;
        [skew_b,angle]=logmatirxs(inv(FK_space_sym(robot,joint_angle))*end_fram);
        angle=vpa(angle,4);
        twist_b = skew_b*angle 
        Twist_r(end+1)=norm(twist_b(1:3));
         Twist_t(end+1)=[norm(twist_b(4:6))];
    end
    %%
I=1:q+1;
plot(I,W,'-o');
xlabel('Number of iteration') ;
ylabel('manipulability measure');
title('Redundancy Resolution');
%%
I=1:q+1;
plot(I,Twist_r,'-o');
xlabel('Number of iteration') ;
ylabel('Norm of rotation twist');
title('Redundancy Resolution');
%%
I=1:q+1;
plot(I,Twist_t,'-o');
xlabel('Number of iteration') ;
ylabel('Norm of translation twist');
title('Redundancy Resolution');