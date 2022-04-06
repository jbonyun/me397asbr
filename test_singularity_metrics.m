
clear;
robot = robot_iiwa();
test_kuka_examples;

rand_angles = @() rand(7,1) * 2 * pi;

%% Test singular position: straight up, zero position; 1/3/5/7 are collinear
% This is actually only 4 DOF, I think. Maybe less.

eg_angles = [0 0 0 0 0 0 0];
Js = J_space(robot, eg_angles);

mu1 = J_isotropy(Js);
assert(mu1 > 1e9, 'Isotropy should be infinite for singular position');
mu2 = J_condition(Js);
assert(mu2 > 1e14, 'Condition Number should be infinite for singular position');
mu3 = J_ellipsoid_volume(Js);
assert(mu3 < 1e-10, 'Volume should be zero for singular position');
disp('Zero pos: all good');

%% Test singular position: A2,A4,A6 are zero -- so still straight up; 1/3/5/7 are collinear
% This is actually only 4 DOF, I think. Maybe less.

eg_angles = rand_angles();
eg_angles([2,4,6]) = 0;
Js = J_space(robot, eg_angles);

mu1 = J_isotropy(Js);
assert(mu1 > 1e9, 'Isotropy should be infinite for singular position');
mu2 = J_condition(Js);
assert(mu2 > 1e14, 'Condition Number should be infinite for singular position');
mu3 = J_ellipsoid_volume(Js);
assert(mu3 < 1e-10, 'Volume should be zero for singular position');
disp('Straight up: all good');

%% Test singular position: A2,A6 are zero; 1/3 are collinear, 5/7 are collinear.
% Yuewan's symbolic says this is singular

eg_angles = rand_angles();
eg_angles([2,6]) = 0;
Js = J_space(robot, eg_angles);

mu1 = J_isotropy(Js);
assert(mu1 > 1e9, 'Isotropy should be infinite for singular position');
mu2 = J_condition(Js);
assert(mu2 > 1e14, 'Condition Number should be infinite for singular position');
mu3 = J_ellipsoid_volume(Js);
assert(mu3 < 2e1, 'Volume should be zero for singular position');
disp('Singular 2/6=0: all good');

%% Test non-singular position: 3,4,5=0
%   Yuewan's symbolic works says rank is still 6, so not singular.
%   I'm double-checking with her becuase it seems to have rank 5 for me...

eg_angles = rand_angles();
eg_angles([3,4,5]) = 0;
Js = J_space(robot, eg_angles);

mu1 = J_isotropy(Js);
%assert(all(~isinf(mu1)), 'Isotropy should finite for non-singular position');
mu2 = J_condition(Js);
%assert(all(~isinf(mu2)), 'Condition Number be finite for non-singular position');
mu3 = J_ellipsoid_volume(Js);
%assert(all(mu3 ~= 0), 'Ellipse volume should be non-zero for non-singular position');
%disp('Non-singular 3/4/5=0: all good');

%% Test singular poses in our examples from real robot
% 1, 2, 3 are so close that we should identify them as singular
% 9 is negative def, very slightly, so probably numerical problem
eg_rows = [1 2 3 8 10 11];  
for eg_i = 1:numel(eg_rows)
    eg_angles = deg2rad(joints(eg_rows(eg_i), :));
    Js = J_space(robot, eg_angles);
    mu1 = J_isotropy(Js);
    assert(mu1 > 1e9, 'Isotropy should be infinite for singular position');
    mu2 = J_condition(Js);
    assert(mu2 > 1e14, 'Condition Number should be infinite for singular position');
    mu3 = J_ellipsoid_volume(Js);
    assert(mu3 < 1e1, 'Volume should be zero for singular position');
end
disp('Singular: all good');

%% Test non-singular poses in our examples from real robot
eg_rows = [4 5 6 7 12 13 14 15 16];
for eg_i = 1:numel(eg_rows)
    eg_angles = deg2rad(joints(eg_rows(eg_i), :));
    Js = J_space(robot, eg_angles);
    mu1 = J_isotropy(Js);
    assert(all(~isinf(mu1)), 'Isotropy should finite for non-singular position');
    mu2 = J_condition(Js);
    assert(all(~isinf(mu2)), 'Condition Number be finite for non-singular position');
    mu3 = J_ellipsoid_volume(Js);
    assert(all(mu3 > 0), 'Ellipse volume should be non-zero for non-singular position');
end
disp('Non-singular: all good');

%% Test non-singular poses with pure random angles.
% In theory we could accidentally get a random singular position. But that
% seems highly unlikely. It takes special effort to create a singularity,
% especially in a 7-DOF robot.
num_reps = 1000;
for i = 1:num_reps
    eg_angles = rand_angles();
    Js = J_space(robot, eg_angles);
    mu1 = J_isotropy(Js);
    assert(all(~isinf(mu1)), 'Isotropy should finite for non-singular position');
    mu2 = J_condition(Js);
    assert(all(~isinf(mu2)), 'Condition Number be finite for non-singular position');
    mu3 = J_ellipsoid_volume(Js);
    assert(all(mu3 ~= 0), 'Ellipse volume should be non-zero for non-singular position');
end
disp('Random: all good');

%% Test near singular positions to examine metric slope
% As we move a joint away from singularity, the metrics ought to improve.

eg_pose = [0 0 0 0 0 0 0];
mu1s = [];
mu2s = [];
mu3s = [];
for delta = 0:0.0001:0.0100
    eg_pose(4) = delta;
    eg_pose(6) = delta;
    Js = J_space(robot, eg_pose);
    mu1s = [mu1s; J_isotropy(Js)];
    mu2s = [mu2s; J_condition(Js)];
    mu3s = [mu3s; J_ellipsoid_volume(Js)];
end
assert(issorted(mu1s, 'descend'), 'Isotropy did not diminish while moving away from singularity');
assert(issorted(mu2s, 'descend'), 'Condition number did not diminish while moving away from singularity');
assert(issorted(mu3s, 'ascend'), 'Ellipsoid volume did not increase while moving away from singularity');
disp('Slope: all good');

%% Another metric slope near singularity

eg_pose = rand_angles();
eg_pose([2 4 6]) = 0;
mu1s = [];
mu2s = [];
mu3s = [];
for delta = 0:0.0001:0.0100
    eg_pose([2 6]) = -delta;
    eg_pose(4) = delta;
    Js = J_space(robot, eg_pose);
    mu1s = [mu1s; J_isotropy(Js)];
    mu2s = [mu2s; J_condition(Js)];
    mu3s = [mu3s; J_ellipsoid_volume(Js)];
end
assert(issorted(mu1s, 'descend'), 'Isotropy did not diminish while moving away from singularity');
assert(issorted(mu2s, 'descend'), 'Condition number did not diminish while moving away from singularity');
assert(issorted(mu3s, 'ascend'), 'Ellipsoid volume did not increase while moving away from singularity');
disp('Slope v2: all good');