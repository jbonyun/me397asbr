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
%joint_angles=[0.2;0.2;0.2;0.2;0.2;0.2;0.2];
joint_angles=[-0.2;-0.2;-0.2;-0.2;-0.2;-0.2;-0.2];
%joint_angles=[0.4;0.4;0.4;0.6;0.9;0.4;0.4];
J = sym('a%d%d',[6,7]);
J(:, 1) = robot.screw(:, 1);
prod_expon = expm_sym(robot.screw(:, 1) , joint_angles(1));
for i = 2:robot.dof
    ad=adjoint_sym(prod_expon);
    J(:, i) = ad * robot.screw(:, i);
    prod_expon = prod_expon * expm_sym(robot.screw(:, i) , joint_angles(i));
end

end_fram=FK_space(robot,joint_angles);
%%
joint_angles=[0.1;0.1;0.1;0.1;0.1;0.1;0.1];
do_print=true;
Tsd=end_fram;
[screw, angle]=logmatirxs(inv(FK_space(robot,joint_angles))*Tsd);
twist_b=screw*angle;
iter=0;
deltaq=0.1;
lr=0.5;
k0=6e-6;
while norm(twist_b(1:3)) > 0.1 || norm(twist_b(4:6)) > 0.1
        iter = iter + 1;
        Jb = J_body(robot, joint_angles);
        % Calculate partial derivative of manipulability wrt joints
        w0 = sqrt(det(Jb*Jb'));
        q0_dot = nan(7,1);
        for i=1:7
            q0 = joint_angles;
            q0(i) = q0(i) + deltaq;
            Ji = J_body(robot,q0);
            wi= sqrt(det(Ji*Ji'));
            q0_dot(i) = (wi - w0) / deltaq;
        end
        % Calculate step
        %(eye(7,7) - J_daggr *Jb) * k0 * q0_dot
        J_daggr = dagger_J(Jb,7,6);
        step = J_daggr * twist_b * lr + (eye(7,7) - J_daggr *Jb) * k0 * q0_dot;
       
        % Update the solution
        joint_angles =joint_angles + step;
        % Prepare state for next iteration
        [screw, angle]=logmatirxs(inv(FK_space(robot,joint_angles))*Tsd);
        twist_b=screw*angle;
        % Save progress
        iter_errang(iter+1) = norm(twist_b(1:3));
        iter_errlin(iter+1) = norm(twist_b(4:6));
        iter_cond(iter+1) = J_condition(Jb);
        iter_stepnorm(iter+1) =  norm(step);
        % Print progress
        if do_print && mod(iter, 1) == 0
            fprintf('%4d  %8f  %9f  %10f  %12.2f  %s\n', iter, iter_stepnorm(iter+1), iter_errang(iter+1), iter_errlin(iter+1), iter_cond(iter+1), mat2str(joint_angles', 4));
        end
    end
%%
clc;
joint_angles=[0.1;0.1;0.1;0.1;0.1;0.1;0.1];
redundancy_resolution(robot, joint_angles, end_fram)

%%
joint_angles=[0.1;0.1;0.1;0.1;0.1;0.1;0.1];
J_inverse_kinematics(robot, joint_angles, end_fram)