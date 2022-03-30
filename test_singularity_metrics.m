

clear;
robot = robot_iiwa();
test_kuka_examples;

rand_angles = @() rand(7,1) * 2 * pi;

%% Test singular position: straight up, zero position; 1/3/5/7 are collinear
% This is actually only 4 DOF, I think. Maybe less.

eg_angles = [0 0 0 0 0 0 0];
Js = J_space(robot, eg_angles);

mu1 = J_isotropy(Js);
assert(any(isinf(mu1)), 'At least one Isotropy should be infinite for singular position');
mu2 = J_condition(Js);
assert(any(isinf(mu2)), 'At least one Condition Number should be infinite for singular position');
mu3 = J_ellipsoid_volume(Js);
assert(any(mu3 == 0), 'At least one volume should be zero for singular position');

%% Test singular position: A2,A4,A6 are zero -- so still straight up; 1/3/5/7 are collinear
% This is actually only 4 DOF, I think. Maybe less.

eg_angles = rand_angles();
eg_angles([2,4,6]) = 0;
Js = J_space(robot, eg_angles);

mu1 = J_isotropy(Js);
assert(any(isinf(mu1)), 'At least one Isotropy should be infinite for singular position');
mu2 = J_condition(Js);
assert(any(isinf(mu2)), 'At least one Condition Number should be infinite for singular position');
mu3 = J_ellipsoid_volume(Js);
assert(any(mu3 == 0), 'At least one volume should be zero for singular position');

%% Test singular position: A2,A6 are zero; 1/3 are collinear, 5/7 are collinear.
% Uh, no, this is apparently not singular...
%   TODO: figure out if this ought to be singular or not.

eg_angles = rand_angles();
eg_angles([2,6]) = 0;
Js = J_space(robot, eg_angles);

mu1 = J_isotropy(Js);
assert(any(isinf(mu1)), 'At least one Isotropy should be infinite for singular position');
mu2 = J_condition(Js);
assert(any(isinf(mu2)), 'At least one Condition Number should be infinite for singular position');
mu3 = J_ellipsoid_volume(Js);
assert(any(mu3 == 0), 'At least one volume should be zero for singular position');

%% Test singular position: A3, A4, A5 are zero -- 2/4/6 are parallel and coplanar + 3/5 are colinear.
% Uh, no, this is apparently not singular...
%   TODO: figure out if this ought to be singular or not.

eg_angles = rand_angles();
eg_angles([3,4,5]) = 0;
Js = J_space(robot, eg_angles);

mu1 = J_isotropy(Js);
assert(any(isinf(mu1)), 'At least one Isotropy should be infinite for singular position');
mu2 = J_condition(Js);
assert(any(isinf(mu2)), 'At least one Condition Number should be infinite for singular position');
mu3 = J_ellipsoid_volume(Js);
assert(any(mu3 == 0), 'At least one volume should be zero for singular position');

%% Test singular poses in our examples from real robot
% 1 is close
% 9 is negative def, very slightly, so probably numerical problem
eg_rows = [8 10 11];  
for eg_i = 1:numel(eg_rows)
    eg_angles = deg2rad(joints(eg_rows(eg_i), :));
    Js = J_space(robot, eg_angles);
    mu1 = J_isotropy(Js);
    assert(any(isinf(mu1)), 'At least one Isotropy should be infinite for singular position');
    mu2 = J_condition(Js);
    assert(any(isinf(mu2)), 'At least one Condition Number should be infinite for singular position');
    mu3 = J_ellipsoid_volume(Js);
    assert(any(mu3 == 0), 'At least one volume should be zero for singular position');
end

%% Test non-singular poses in our examples from real robot
eg_rows = [1 2 3 4 5 6 7 12 13 14 15 16];  % 1 is close
for eg_i = 1:numel(eg_rows)
    eg_angles = deg2rad(joints(eg_rows(eg_i), :));
    Js = J_space(robot, eg_angles);
    mu1 = J_isotropy(Js);
    assert(all(~isinf(mu1)), 'Isotropy should both be finite for non-singular position');
    mu2 = J_condition(Js);
    assert(all(~isinf(mu2)), 'Condition Number should both be finite for non-singular position');
    mu3 = J_ellipsoid_volume(Js);
    assert(all(mu3 ~= 0), 'Ellipse volumes should both be non-zero for non-singular position');
end
