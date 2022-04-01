% Support for calculating the Jacobian symbolically

clear;

robot = robot_iiwa();

theta = sym('theta', [7 1], {'real'});
assumeAlso(theta>0 & theta<2*pi);  % Assumptions necessary for simplify to work efficiently.

% Example for second column
% Matches what I did by hand
%J2 = simplify(adjoint_transform(expm(skewsym(robot.screw(:,1)) * theta(1))) * robot.screw(:,2));

% Do the whole thing!
J = nan([6 7], 'sym');
for i = 1:7
    prod = eye(4, 'sym');
    for j = 1:(i-1)
        prod = prod * expm(skewsym(robot.screw(:,j)) * theta(j));
    end
    J(:,i) = adjoint_transform(prod) * robot.screw(:,i);
    J(:,i) = simplify(J(:,i));  % Will turn exponentials into sin/cos.

    fprintf('Column %d\n', i);
    for j = 1:6
        fprintf('%s\n', symbolic_c_s_notation(J(j,i)));
    end
end

%% Do in body frame

Jb = nan([6 7], 'sym');
% First column is just the screw for first joint
Jb(:, robot.dof) = robot.bscrew(:, robot.dof);
% Keep a running product of the forward kinematics from end to start
prod_expon = expm(-skewsym(robot.bscrew(:, robot.dof)) * theta(robot.dof));
for i = (robot.dof-1):-1:1
    % Calculate this column of J
    Jb(:,i) = adjoint_transform(prod_expon) * robot.bscrew(:, i);
    % Update running product of FK of joints we have passed
    prod_expon = prod_expon * expm(-skewsym(robot.bscrew(:, i)) * theta(i));
end

fprintf('\nBody Frame Jacobian\n');
for i = 1:7
    fprintf('\nColumn %d\n', i);
    for j = 1:6
        fprintf('%s\n', symbolic_c_s_notation(Jb(j,i)));
    end
end