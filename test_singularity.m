clear;  
syms x1 x2 x3 x4 x5 x6 x7
syms d1 d2 d3 d4 d5 real

robot.home = [eye(3,3) [0; 0; d1+d2+d3+d4+d5]; 0 0 0 1];
    
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
        0 0 d1;
        0 0 0;
        0 0 d1+d2+d3;
        0 0 0;
        0 0 d1+d2+d3+d4+d5;
        0 0 0;
        ];

%     robot.offset = [
%         0 0 0;
%         0 0 360;
%         0 0 0;
%         0 0 780;
%         0 0 0;
%         0 0 1180;
%         0 0 0;
%         ];
    % Extract DOF from the parameters we have already given.
    robot.dof = size(robot.axes,1);
    
    % Calculate the screw vectors from each joint's axis at home and translation
    % at home.
    for i = 1:robot.dof
        v = -cross(robot.axes(i,:), robot.offset(i,:));
        robot.screw(:, i) = [robot.axes(i,:)'; v'];
    end


%joint_angles=[x1;x2;x3;0;x5;x6;x7];
%joint_angles=[x1;0;x3;x4;x5;0;x7]; %x2=0,x6=0
%joint_angles=[x1;pi/2;x3;0;x5;x6;x7]; %x2=pi/2,x4=0
%joint_angles=[x1;x2;x3;pi/2;0;0;x7];
%joint_angles=[x1;x2;0;x4;pi/2;0;x7]; %x3=0 x5=pi/2,x6=0
%joint_angles=[x1;0;-pi/2;x4;0;pi/2;x7]; %x2=0,x3=-pi/2,x5=0,x6=pi/2
%joint_angles=[0;0;0;0;0;0;0];
%joint_angles=[10;0;10;10;10;0;10];
%joint_angles=[.1;.1;0;0;0;.1;.1];
%joint_angles=[x1;0;x3;x4;x5;0;x7];
joint_angles= [4.5417 2.4359 0 0 0 3.7482 5.8714];
J = sym('a%d%d',[6,7]);
%%
J(:, 1) = robot.screw(:, 1);
prod_expon = expm_sym(robot.screw(:, 1) , joint_angles(1));
for i = 2:robot.dof
    ad=adjoint_sym(prod_expon);
    J(:, i) = ad * robot.screw(:, i);
% Update running product of FK of joints we have passed
    prod_expon = prod_expon * expm_sym(robot.screw(:, i) , joint_angles(i));
end
rank(J)
%%
J
rank(J)

