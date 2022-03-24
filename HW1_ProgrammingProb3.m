% Solution to Problem 3
% Jeff Bonyun (jb79332), jbonyun@utexas.edu, 20220222
% On behalf of the Sun/Bonyun team for ME397 ASBR, Spring 2022.

% Example inputs from problem, defining the screw and the body frame to
% transform.
q = [0; 2; 0];      % offset from {s} origin to a point on screw axis.
shat = [0; 0; 1];   % screw axis wrt {s}
h = 2;              % pitch of screw (linear divided by angular)
theta = pi;         % how far we rotate around screw axis.
T = [1 0 0 2; 0 1 0 0; 0 0 1 0; 0 0 0 1];   % initial {b} configuration wrt {s}


%% Part 1: find final configuration of body T1 wrt {s}

% Since everything in the twist/screw discussion is based on velocities,
% I guess I will assume "1 time unit" of time passes while theta is
% traversed. That gives thetadot = theta.
thetadot = theta;  % speed following screw path??

% Get twist from crossproduct.
V = [shat * thetadot; cross(-shat*thetadot, q) + h*shat*thetadot];
% Turn that into a screw, which is normalized twist.
[S, screwtheta] = twist2screw(V);

% Apply that screw at increasing thetas.
screwthetastep = linspace(0, 1, 5) * screwtheta;
axis_labels = {'$\enspace T$', '$\enspace\theta = \frac{1}{4}\pi$', '$\enspace\theta = \frac{1}{2}\pi$', '$\enspace\theta = \frac{3}{4}\pi$', '$\enspace T_1, \theta = \pi$'};
for i = 1:numel(screwthetastep)
    Tsteps{i} = screw2trans(S, screwthetastep(i)) * T;
end
T1 = Tsteps{end}; % We need this convenient for later.

%% Plot path from T0 to T1

f = figure;
% Plot {s}
plot_3d_axis([0 0 0], [1 0 0], [0 1 0], [0 0 1], 'Label', '$\enspace\{s\}$');
% Plot the progression of T, from initial location to T1.
for i = 1:numel(Tsteps)
    plot_3d_axis_transform(Tsteps{i}, 'Label', axis_labels{i});
end
title('Evolution of T over \pi radians');
xlabel('x'); ylabel('y'); zlabel('z');
% View setup based on manual tweaking.
xlim([-4 4]); ylim([-1 7]); zlim([0 8]);
view(65.5, 50.6);

%% Part 2: find screw axis S1 and theta1 that would take rigid body from T1 to {s}

% From a practical standpoint, we can see that the screw axis needs to be
% vertical (in the +/- z direction) so that the z axis of the two frames
% can remain the same. Knowing that, we can see that it must rotate pi rads
% as it moves, so that the +x and +y directions move 180 degrees. It does
% this as it moves 2*pi in z, so h must be 2 so that it will move this far
% vertically while it rotates pi around the axis. We can find the x-y
% location of the axis by putting it half way between {s} and {T1} in both
% the x and y. T1{s} is at -2,4, so the axis for the screw is at -1,2.
% That's our q (at any z). And we want the direction to be down, which is
% -z, which is (0, 0, -1).
% So:
%   q = (-1, 2, *)
%   direction = (0, 0, -1)
%   h = 2.
%   theta = pi
% But we also have to show how to calculate that...

% Find the rotation that will bring T1 back to T, wrt {s}
% We know the rotation from s to T1 wrt {s}. Invert that.
R_s_to_1_in_s = T1(1:3,1:3);
R_1_to_s_in_s = R_s_to_1_in_s';
% Find the translation that will bring T1 back to T, wrt {s}
% We know s to T1 in {s}, so just negate that.
p_s_to_1_in_s = T1(1:3, 4);
p_1_to_s_in_s = -p_s_to_1_in_s;

% Now we need a way to effect this rotation and translation via screw.

% Find the required rotation in axis-angle representation.
aa = rot2aa(R_1_to_s_in_s);
direction_1_to_s_in_s = aa(1:3);
theta_1_to_s_in_s = aa(4);

% The only way to move along the axis is via h.
% So find the translation needed along axis.
% Project the needed translation onto axis.
how_far_along_axis = dot(p_1_to_s_in_s, direction_1_to_s_in_s);
projected = how_far_along_axis * direction_1_to_s_in_s;
% Find h as how many theta's that is.
h_1_to_s_in_s = how_far_along_axis / theta_1_to_s_in_s;

% The remainder of the required translation must come from rotating about
% the screw.
translation_remainder = p_1_to_s_in_s - projected;
% This translation comes from rotating about axis by theta.
% Find the center of rotation that has this cord and this theta between
% them. Ideally not using the assumption that theta is pi, as it is in our
% example calculation.
midpoint_1_to_s_in_s = translation_remainder / 2 + p_s_to_1_in_s;
perp_to_1s_and_axis = cross(direction_1_to_s_in_s, p_s_to_1_in_s);
perp_to_1s_and_axis = perp_to_1s_and_axis / norm(perp_to_1s_and_axis);
distance_midpoint_to_center = norm(midpoint_1_to_s_in_s) / tan(theta_1_to_s_in_s / 2);
center = midpoint_1_to_s_in_s + distance_midpoint_to_center * perp_to_1s_and_axis;
q = center;

% Now we have the geometric screw:
% q
% direction_1_to_s_in_s
% theta_1_to_s_in_s
% h_1_to_s_in_s

%% Add screw axis to plot

figure;  % Do you want a new plot, or add to old plot?
hold on;
plot_3d_axis([0 0 0], [1 0 0], [0 1 0], [0 0 1], 'Label', '$\enspace\{s\}$');
plot_3d_axis_transform(T1, 'Label', '$T_1$');
plot_screw(q, direction_1_to_s_in_s, h_1_to_s_in_s, theta_1_to_s_in_s);
title('Screw to bring T_1 to \{s\}');
xlabel('x'); ylabel('y'); zlabel('z');
xlim([-4 4]); ylim([-1 7]); zlim([0 8]);
view(65.5, 50.6); % View setup based on manual tweaking.