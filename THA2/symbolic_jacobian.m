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
end