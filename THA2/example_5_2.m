clear;

L1 = sym('L1', {'real', 'positive'});
L2 = sym('L2', {'real', 'positive'});
theta1 = sym('theta1', {'real'});
theta2 = sym('theta2', {'real'});
theta3 = sym('theta3', {'real'});
theta4 = sym('theta4', {'real'});
assumeAlso(theta1 >= 0 & theta1 < 2*pi);
assumeAlso(theta2 >= 0 & theta2 < 2*pi);
assumeAlso(theta3 >= 0 & theta3 < 2*pi);

w1 = [0 0 1]';
q1 = [0 0 0]';
S1 = [w1; -cross(w1, q1)]; % = [0 0 1 0 0 0]';
w2 = [0 0 1]';
q2 = [L1 0 0]';
S2 = [w2; -cross(w2, q2)]; % = [0 0 1 0 -L1 0]';
w3 = [0 0 1]';
q3 = [L1+L2 0 0]';
S3 = [w3; -cross(w3, q3)]; % = [0 0 1 0 -L1-L2 0]';
w4 = [0 0 0]';
q4 = [0 0 1]';
S4 = [w4; q4]; % = [0 0 0 0 0 1]';

S = [S1 S2 S3 S4]

J1 = simplify(vpa(S1));
J2 = simplify(vpa(adjoint_transform(expm(skewsym(S1) * theta1)) * S2));
J3 = simplify(vpa(adjoint_transform(expm(skewsym(S1) * theta1) * expm(skewsym(S2) * theta2)) * S3));
J4 = simplify(vpa(adjoint_transform(expm(skewsym(S1) * theta1) * expm(skewsym(S2) * theta2) * expm(skewsym(S3) * theta3)) * S4));

J = [J1 J2 J3 J4]


%% The code from J_space, but using given screws...

    clear J;
    joint_angles = [theta1 theta2 theta3 theta4]';
    % First column is just the screw for first joint
    J(:, 1) = S(:, 1);
    % Keep a running product of the forward kinematics
    prod_expon = expm(skewsym(S(:, 1)) * joint_angles(1));
    for i = 2:4
        % Calculate this column of J
        J(:, i) = adjoint_transform(prod_expon) * S(:, i);
        % Update running product of FK of joints we have passed
        prod_expon = prod_expon * expm(skewsym(S(:, i)) * joint_angles(i));
    end
    simplify(J)