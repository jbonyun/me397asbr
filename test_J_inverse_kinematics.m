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


%joint_angles=[0.5;0.5;0.5;0.5;0.5;0.5;0.5];
joint_angles=[0.6;0.6;0.6;0.6;0.6;0.6;0.6];
J = sym('a%d%d',[6,7]);
J(:, 1) = robot.screw(:, 1);
prod_expon = expm_sym(robot.screw(:, 1) , joint_angles(1));
for i = 2:robot.dof
    ad=adjoint_sym(prod_expon);
    J(:, i) = ad * robot.screw(:, i);
    prod_expon = prod_expon * expm_sym(robot.screw(:, i) , joint_angles(i));
end

end_fram=FK_space_sym(robot,joint_angles);


%%
kuka = importrobot('iiwa14.urdf');
config = homeConfiguration(kuka);


    joint_angle=[0.2,0.2,0.2,0.2,0.2,0.2,0.2];
    for i=1:7
            config(i).JointPosition = double(joint_angle(i));
    end
    show(kuka,config);
    %joint_angle=[0.3,0.3,0.3,0.3,0.3,0.3,0.3];
    joint_angle=joint_angle'
    i=0;
    [skew_b,angle]=logmatirxs(inv(FK_space(robot,joint_angle))*end_fram);
    angle=vpa(angle,4);
    twist_b = skew_b*angle
    while norm(twist_b(1:3))>0.05 || norm(twist_b(4:6))>0.05
        Jb=J_body(robot, joint_angle);
        J_daggr=dagger_J(Jb,7,6);
        joint_angle=joint_angle+ J_daggr*twist_b
        
        for i=1:7
            config(i).JointPosition = double(joint_angle(i));
        end
        show(kuka,config);
        
        i=i+1;
        [skew_b,angle]=logmatirxs(inv(FK_space(robot,joint_angle))*end_fram);
        angle=vpa(angle,4);
        twist_b = skew_b*angle 
    end

%%

   joint_angle=[0.2,0.2,0.2,0.2,0.2,0.2,0.2];
    for i=1:7
            config(i).JointPosition = double(joint_angle(i));
    end

    %joint_angle=[0.3,0.3,0.3,0.3,0.3,0.3,0.3];
    joint_angle=joint_angle'
    i=0;
    [skew_b,angle]=logmatirxs(inv(FK_space(robot,joint_angle))*end_fram);
    angle=vpa(angle,4);
    twist_b = skew_b*angle
    while norm(twist_b(1:3))>0.05 || norm(twist_b(4:6))>0.05
        Jb=J_body(robot, joint_angle);
        J_daggr=dagger_J(Jb,7,6);
        joint_angle=joint_angle+J_daggr*twist_b
        i=i+1;
        [skew_b,angle]=logmatirxs(inv(FK_space(robot,joint_angle))*end_fram);
        angle=vpa(angle,4);
        twist_b = skew_b*angle 
    end


