
addpath('..');
addpath('THA4');

clear;
robot = robot_iiwa();
test_kuka_examples;

%% Explore IK examples
% Each example has a starting guess and a target, in angles (so we know are possible).
% The IK is run to get the cartesian pose of the target angles.
% Result is successful if pose from IK is very close to target pose.

test_cases = {};
%case_desc = 'to zero position'; test_cases{end + 1} = {[0 .1 0 .1 0 0 0]', [0 0 0 0 0 0 0]'}; % Start non-singularity, move to singularity.
%case_desc = 'to zero position'; case_fname = 'AtoZeroPos'; test_cases{end + 1} = {[0 .3 -.2 .3 0 .5 0]', [0 0 0 0 0 0 0]'}; % Start non-singularity, move to singularity.
%test_cases{end + 1} = {[0 0 0 0 0 0 0]', [0 -.1 .1 -.1 0 0 0]'}; % Start at singularity, move away.
%test_cases{end + 1} = {[0 .1 0 .1 0 0 0]', [0 -.1 0 -.1 0 0 0]'}; % Direct path is through singularity.
%test_cases{end + 1} = {[0 .1 .1 .1 0 0 0]', [0 -.1 .1 -.1 0 0 0]'};
%case_desc = '+0.1 to -0.1 in all joints'; case_fname = 'XXthroughZero'; test_cases{end + 1} = {[.1 .1 .1 .1 .1 .1 .1]', [-.1 -.1 -.1 -.1 -.1 -.1 -.1]'};
%case_desc = '+0.1 to -0.2 in all joints'; case_fname = 'AthroughZero'; test_cases{end + 1} = {[.1 .1 .1 .1 .1 .1 .1]', [-.2 -.2 -.2 -.2 -.2 -.2 -.2]'};
%case_desc = '+0.4 to -0.4 in all joints'; case_fname = 'CthroughZero'; test_cases{end + 1} = {[.4 .4 .4 .4 .4 .4 .4]', [-.4 -.4 -.4 -.4 -.4 -.4 -.4]'};
%case_desc = 'To off-zero singularity'; case_fname = 'XXtoSing'; test_cases{end + 1} = {[.4 .4 .4 .4 .4 .4 .4]', [.4 0 .4 .4 .4 0 .4]'};
%case_desc = 'Through off-zero singularity'; case_fname = 'XXthroughSing'; test_cases{end + 1} = {[.4 .4 .4 .4 .4 .4 .4]', [.4 -0.4 .4 .4 .4 -0.4 .4]'};
%case_desc = 'To off-zero singularity'; case_fname = 'XXtoSing'; test_cases{end + 1} = {[1 1 1 1 1 1 1]', [1 0 1 1 1 0 1]'};
case_desc = 'To off-zero singularity B'; case_fname = 'XXtoSing'; test_cases{end + 1} = {[1 1 1 1 1 1 1]', [1 pi/2 1 0 1 1 1]'};
%case_desc = 'Through off-zero singularity B'; case_fname = 'XXthroughSing'; test_cases{end + 1} = {[0 1 1 1 1 1 1]', [0 pi/2+.1 1 0-0.15 1 1 1]'};
%case_desc = 'Through off-zero singularity'; case_fname = 'BthroughSing'; test_cases{end + 1} = {[1 1 1 1 1 1 1]', [1 -1 1 1 1 -1 1]'};
%test_cases{end + 1} = {[-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3]', [0.1;0.1;0.1;0.1;0.1;0.1;0.1]};
%case_desc = 'folding up'; case_fname = 'CfoldingUp'; test_cases{end + 1} = {deg2rad(joints(5,:))', [-4.87125941617731 -0.0735561342818407 0.63521184011302 4.18870325095684 1.91282383432598 3.3090539278487 -2.35266570423377]'}; % Starts at a singularity
%case_desc = 'random poses'; case_fname = 'XXInteresting'; test_cases{end + 1} = {deg2rad(joints(12,:))', [3.60426218545767 1.26563657105162 -3.19483613264846 -6.92175875711803 -1.79462324164591 0.672289765429706 2.96056445057525]'};
%test_cases{end + 1} = {deg2rad(joints(12,:))', deg2rad(joints(12,:))' + randn(size(joints,2), 1) * 1 * pi};
%test_cases{end + 1} = {deg2rad(joints(12,:))' + randn(size(joints,2), 1) * 1 * pi, deg2rad(joints(12,:))' + randn(size(joints,2), 1) * 1 * pi};
%case_desc = 'Yuewans plane test'; case_fname = 'PlaneTest'; test_cases{end + 1} = {[0.2 0.2 0.2 0.2 0.2 0.2 0.2]', [1 -1 1 1 1 -1 1]'};
%case_desc = 'Test Joint Limits'; case_fname = 'TestJointLimits'; rng(12346); test_cases{end + 1} = {robot.joint_limits(:,1) + rand(robot.dof, 1) .* (robot.joint_limits(:,2) - robot.joint_limits(:,1)), robot.joint_limits(:,1) + rand(robot.dof, 1) .* (robot.joint_limits(:,2) - robot.joint_limits(:,1))};
%case_desc = 'Demo 3mm Sphere'; case_fname = 'Demo3mm'; rng(12350); test_cases{end + 1} = {robot.joint_limits(:,1) + rand(robot.dof, 1) .* (robot.joint_limits(:,2) - robot.joint_limits(:,1)), robot.joint_limits(:,1) + rand(robot.dof, 1) .* (robot.joint_limits(:,2) - robot.joint_limits(:,1))};
%case_desc = 'Demo Straight Up'; case_fname = 'Straight Up'; test_cases{end + 1} = {[0 0 0 0 0 0 0]', [0 0 0 0 0 0 0]'};
%case_desc = 'Demo Orientation'; case_fname = 'DemoOr'; rng(12355); test_cases{end + 1} = {robot.joint_limits(:,1) + rand(robot.dof, 1) .* (robot.joint_limits(:,2) - robot.joint_limits(:,1)), robot.joint_limits(:,1) + rand(robot.dof, 1) .* (robot.joint_limits(:,2) - robot.joint_limits(:,1))};
%case_desc = 'Demo Plane'; case_fname = 'DemoPlane'; rng(12361); test_cases{end + 1} = {robot.joint_limits(:,1) + rand(robot.dof, 1) .* (robot.joint_limits(:,2) - robot.joint_limits(:,1)), robot.joint_limits(:,1) + rand(robot.dof, 1) .* (robot.joint_limits(:,2) - robot.joint_limits(:,1))};
%rng(12362); case_desc = 'Random To Random'; case_fname = 'Random'; test_cases{end + 1} = {robot.joint_limits(:,1) + rand(robot.dof, 1) .* (robot.joint_limits(:,2) - robot.joint_limits(:,1)), robot.joint_limits(:,1) + rand(robot.dof, 1) .* (robot.joint_limits(:,2) - robot.joint_limits(:,1))};
%case_desc = 'Yuewans sphere test'; case_fname = 'YSphereTest'; test_cases{end + 1} = {[0.2 0.2 0.2 0.2 0.2 0.2 0.2]', [1.739 4.61 -2.44 1 -2.262 4.638 1.842]'};



%method_name = 'Without Joint Limits'; method_fname = 'ConstrainedNoJointLimits'; lr = 0.5; step_function = @(ang, tw) constrained_IK_step(robot, ang, tw, nan, lr, 'use_joint_limits', false);
%method_name = 'With Joint Limits'; method_fname = 'ConstrainedJointLimits'; lr=0.5; step_function = @(ang, tw) constrained_IK_step(robot, ang, tw, nan, lr, 'use_joint_limits', true);

%method_name = 'Just Kinetic Energy'; method_fname = 'JustKinetic'; lr = 1.0; step_function = @(ang, tw) constrained_IK_step(robot, ang, tw, nan, lr, 'use_joint_limits', false, 'max_distance_from_goal', nan, 'weight_loc', 0, 'weight_kinetic', 1);
%method_name = 'Max 3mm'; method_fname = 'Max3mm'; lr = 1.0; step_function = @(ang, tw) constrained_IK_step(robot, ang, tw, nan, lr, 'use_joint_limits', false, 'max_distance_from_goal', 3, 'weight_loc', 0, 'weight_kinetic', 1);
%method_name = 'Max 4mm'; method_fname = 'Max4mm'; lr = 1.0; step_function = @(ang, tw) constrained_IK_step(robot, ang, tw, nan, lr, 'use_joint_limits', false, 'max_distance_from_goal', 4, 'weight_loc', 0, 'weight_kinetic', 1);
%method_name = 'Max 10mm'; method_fname = 'Max10mm'; lr = 1.0; step_function = @(ang, tw) constrained_IK_step(robot, ang, tw, nan, lr, 'use_joint_limits', false, 'max_distance_from_goal', 10, 'weight_loc', 0, 'weight_kinetic', 1);

%method_name = 'w_{\alpha}=50'; method_fname = 'Orientation50'; lr=0.1; step_function = @(ang, tw) constrained_IK_step(robot, ang, tw, nan, lr, 'use_joint_limits', true, 'weight_orientation', 50, 'weight_kinetic', 1, 'joint_vel_limit', 0.2);
%method_name = 'w_{\alpha}=5'; method_fname = 'Orientation5'; lr=0.25; step_function = @(ang, tw) constrained_IK_step(robot, ang, tw, nan, lr, 'use_joint_limits', true, 'weight_orientation', 5, 'weight_kinetic', 1, 'joint_vel_limit', nan);
%method_name = 'w_{\alpha}=1'; method_fname = 'Orientation1'; lr=0.25; step_function = @(ang, tw) constrained_IK_step(robot, ang, tw, nan, lr, 'use_joint_limits', true, 'weight_orientation', 1, 'weight_kinetic', 1, 'joint_vel_limit', nan);
%method_name = 'Orientation=5,Centering=5'; method_fname = 'Orientation5Center5'; lr=0.1; step_function = @(ang, tw) constrained_IK_step(robot, ang, tw, nan, lr, 'use_joint_limits', true, 'weight_orientation', 5, 'weight_kinetic', 1, 'joint_vel_limit', nan, 'weight_jointcenter', 5);
%method_name = 'w_{\alpha}=off'; method_fname = 'OrientationOff'; lr=0.25; step_function = @(ang, tw) constrained_IK_step(robot, ang, tw, nan, lr, 'use_joint_limits', true, 'weight_orientation', 0, 'weight_kinetic', 1, 'joint_vel_limit', nan);
%method_name = 'w_{\alpha}=off'; method_fname = 'OrientationOff'; lr=0.1; step_function = @(ang, tw) constrained_IK_step(robot, ang, tw, nan, lr, 'use_joint_limits', true, 'weight_orientation', 0, 'weight_kinetic', 1, 'joint_vel_limit', 0.2);

%method_name = 'Plane Limit On'; method_fname = 'PlaneOn'; lr=1; step_function = @(ang, tw) constrained_IK_step(robot, ang, tw, nan, lr, 'use_joint_limits', true, 'weight_kinetic', 1, 'joint_vel_limit', 0.2, 'plane', [-.3 -.5 -.9; 250 550 600]');
%method_name = 'Plane Limit Off'; method_fname = 'PlaneOff'; lr=1; step_function = @(ang, tw) constrained_IK_step(robot, ang, tw, nan, lr, 'use_joint_limits', true, 'weight_kinetic', 1, 'joint_vel_limit', 0.2, 'plane', [-.3 -.5 -.9; 250 550 600]', 'enforce_plane', false);

%method_name = 'w_{\alpha}=50, Plane Limit Off'; method_fname = 'Orientation50PlaneOff'; lr=.333; step_function = @(ang, tw) constrained_IK_step(robot, ang, tw, nan, lr, 'use_joint_limits', true, 'weight_orientation', 50, 'weight_kinetic', 1, 'joint_vel_limit', 0.2, 'plane', [-.35 -.55 -.8; 250 550 600]', 'enforce_plane', false);
%method_name = 'w_{\alpha}=50, Plane Limit On'; method_fname = 'Orientation50PlaneOn'; lr=0.333; step_function = @(ang, tw) constrained_IK_step(robot, ang, tw, nan, lr, 'use_joint_limits', true, 'weight_orientation', 50, 'weight_kinetic', 1, 'joint_vel_limit', 0.2, 'plane', [-.35 -.55 -.8; 250 550 600]', 'enforce_plane', true);

method_name = 'Best Method'; method_fname = 'BestMethod'; lr = 0.333; step_function = @(ang, tw) constrained_IK_step(robot, ang, tw, nan, lr, 'use_joint_limits', true, 'max_distance_from_goal', 3, 'weight_orientation', 50, 'weight_loc', 1, 'weight_kinetic', 1, 'joint_vel_limit', 0.2);

plot_subtitle = sprintf('%s, lr=%.2f ', case_desc, lr);
video_fname = sprintf('video_%s_%s_lr%4.2f.mp4', method_fname, case_fname, lr);
do_plot_details = true;

for i_test_cases = 1:numel(test_cases)
    %i = test_cases(i_test_cases);
    joint_angles = test_cases{i_test_cases}{1};
    % Find starting pose.
    start_pose = FK_space(robot, joint_angles);
    % Invent a destination pose.
    target_joint_angles = test_cases{i_test_cases}{2};
    targeteeTs = FK_space(robot, target_joint_angles);
    tool_vector_in_body = [0; 0; 100];
    toolTb = rottranslation2trans(eye(3), tool_vector_in_body);
    targettoolTs = targeteeTs * toolTb;
    % Replace target orientation with starting orientation, because we're
    % trying to work on translation and stability.
    targettoolTs(1:3, 1:3) = trans2rot(start_pose);
    init_guess = joint_angles;
    fprintf('\nTwist from %s to %s\nJoints from %s to (for example) %s\n', mat2str(trans2twist(start_pose)', 5), mat2str(trans2twist(targettoolTs)', 5), mat2str(init_guess', 5), mat2str(target_joint_angles', 5));
    % Now solve it.
    [ik_angles, iter_errang, iter_errlin, iter_cond, iter_step, iter_stepnorm] = inverse_kinematics_movie(robot, init_guess, targettoolTs, step_function, do_plot_details, method_name, plot_subtitle, video_fname, lr);
    ik_pose = FK_space(robot, ik_angles);
    % Print results
    fprintf('Max joint velocity: %.2f %s\n', max(max(abs(iter_step))) ./ lr, mat2str(max(abs(iter_step)) ./ lr, 3));
    fprintf('Joint Angle Solution: %s\n', mat2str(ik_angles', 5));
    fprintf('Angular error in ZYZ: %s\n', mat2str((rot2zyz(trans2rot(targettoolTs))-rot2zyz(trans2rot(ik_pose)))', 5));
    fprintf('Linear error in xyz: %s\n', mat2str((trans2translation(targettoolTs)-trans2translation(ik_pose))', 5));
    if ~(all(rot2zyz(trans2rot(targettoolTs))-rot2zyz(trans2rot(ik_pose)) < 1e-1, 'all') || all(rot2rpy(trans2rot(targettoolTs))-rot2rpy(trans2rot(ik_pose)) < 1e-1, 'all'))
        fprintf('** Rotation not converged **\n');
    end
    if ~(all((trans2translation(targettoolTs)-trans2translation(ik_pose)) < 1e-0, 'all'))
        fprintf('** Translation not converged **\n');
    end
end