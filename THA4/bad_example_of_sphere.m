
clear;

% Calculate the unit polyhedron
m = 10; n = 10;  % Count of polygon mesh; bigger is a better approximation.
[polyalpha,polybeta] = meshgrid(linspace(0, 2*pi * (1 - 1/m), m), linspace(0, 2*pi * (1 - 1/n), n));
polyalpha = reshape(polyalpha, [], 1);
polybeta = reshape(polybeta, [], 1);
polyhedron = [cos(polyalpha).*cos(polybeta) cos(polyalpha).*sin(polybeta) sin(polyalpha)];
clear -polyhedron;  % Clear everythign except polyhedron.

% Start robot straight up.
robot = robot_iiwa();
joint_angles = [0; 0; 0; 0; 0; 0; 0];
Js = J_space(robot, joint_angles);
Jseps = Js(4:6, :);
polyA = polyhedron * Jseps;

% Test tiny moves in the non-Z axes (the others don't affect translation in
% this starting position, because they all just rotate around Z axis).
dq_small_A2 = [0; 0.01; 0; 0; 0; 0; 0];
dq_small_A4 = [0; 0; 0; 0.01; 0; 0; 0];
dq_small_A6 = [0; 0; 0; 0; 0; 0.01; 0];

% How much does that move the tip?
A2_plane_violations = polyA * dq_small_A2;
A4_plane_violations = polyA * dq_small_A4;
A6_plane_violations = polyA * dq_small_A6;

% Find the b for a scenario where we are currently at (0,0,0) and pgoal is
% (0,0,0). So we're already at the center of the sphere.
polyb = repmat(3, size(polyA,1), 1) - polyhedron * ([0 0 0]' - [0 0 0]');
% The answer is 3mm for all of them bc we're already in center.
% I.e. all elements of polyb are 3.

fprintf('Largest plane violation for 0.01 rad change in A2: %.2f\n', max(A2_plane_violations));
fprintf('Largest plane violation for 0.01 rad change in A4: %.2f\n', max(A4_plane_violations));
fprintf('Largest plane violation for 0.01 rad change in A6: %.2f\n', max(A6_plane_violations));

% Note that larger violations happen if you move A6 than if you move A2.
% This is wrong (?) because A2 has a longer arm on it. A2 is near the base,
% so a 0.01 rad change in A2 should move the end effector and tip more than
% 0.01 rad change in A6.