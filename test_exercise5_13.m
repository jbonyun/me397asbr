clear;
syms x1 x2 x3 x4 x5 x6 L real
robot.home = [eye(3,3) [0; 3*L; 0]; 0 0 0 1];
robot.axes = [
        0  0  1;
        0  1  0;
        -1  0  0;
        -1  0  0;
        -1  0  0;
        0  1  0;
        ];
% In home position, the location of the center of each joint
robot.offset = [
        0 0 0;
        0 0 0;
        0 0 0;
        0 L 0;
        0 2*L 0;
        0 0 0;
        ];

robot.dof = size(robot.axes,1);
    

for i = 1:robot.dof
    v = -cross(robot.axes(i,:), robot.offset(i,:));
    robot.screw(:, i) = [robot.axes(i,:)'; v'];
end
    

joint_angles=[x1;x2;x3;x4;x5;x6];
J = sym('a%d%d',[6,6]);
%%
J(:, 1) = robot.screw(:, 1);
prod_expon = expm_sym(robot.screw(:, 1) , joint_angles(1));
for i = 2:robot.dof
    %ad = [prod_expon(1:3,1:3) zeros(3,3); skewsym_sym(prod_expon(1:3,4)) * prod_expon(1:3,1:3) prod_expon(1:3,1:3)];
    ad=adjoint_sym(prod_expon);
    J(:, i) = ad * robot.screw(:, i);
% Update running product of FK of joints we have passed
    prod_expon = prod_expon * expm_sym(robot.screw(:, i) , joint_angles(i));
end

%%
for i=1:6
    J(:,i)
end
%%
J
det(J)
rank(J)

