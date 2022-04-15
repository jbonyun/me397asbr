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

end_fram=FK_space(robot,joint_angles)
%%

joint_angles=[0.1;0.1;0.1;0.1;0.1;0.1;0.1];
Tsd=end_fram;
[screw, angle]=logmatirxs(inv(FK_space(robot,joint_angles))*Tsd);
twist_b=screw*angle;


while norm(twist_b(1:3)) > 0.01 || norm(twist_b(4:6)) > 0.01
    Jacobin=J_body(robot,joint_angles);
    %lama=(twist_b'*(Jacobin*Jacobin'*twist_b))/norm(Jacobin*Jacobin'*twist_b);
    lama=0.000001;
    step=transpose(Jacobin)*twist_b*lama;
    joint_angles = joint_angles + step
    %joint_angles=joint_angles+transpose(Jacobin)*twist_b*lama;
    [screw, angle]=logmatirxs(inv(FK_space(robot,joint_angles))*Tsd);
    twist_b=screw*angle;
    norm(twist_b);

end

%%
joint_angles=[0.1;0.1;0.1;0.1;0.1;0.1;0.1];
J_transpose_kinematics(robot, joint_angles, end_fram)