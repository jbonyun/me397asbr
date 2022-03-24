
clear;

V = [0; 1; 2; 3; 0; 0];

%S = [0; sqrt(5)/5; 2*sqrt(5)/5;  3*sqrt(5)/5; 0; 0];
theta = sqrt(5);
wv = V(1:3);
vv = V(4:6);
shat = wv / theta;
linsolve(-skewsym(shat), vv)

q = [0; 2; 1];
how_far_along_axis = dot(vv/theta, shat);
projected = how_far_along_axis * shat;
% Find h as how many theta's that is.
h = how_far_along_axis / theta;

%plot_screw(q, shat, h, theta);
%plot_3d_arrow(q, shat, 1, 'LineSpec', 'k', 'LineWidth', 1);

S = V ./ theta;
[q, shat, h] = screw2screwgeo(S, theta);

figure;
plot_screw(q, shat, h, theta);
plot_3d_axis_transform([1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 0])
view(65.5, 50.6); % View setup based on manual tweaking.
xlim([-2 5]);
ylim([-2 5]);
zlim([-2 5]);