
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
%case_desc = 'Through off-zero singularity'; case_fname = 'BthroughSing'; test_cases{end + 1} = {[1 1 1 1 1 1 1]', [1 -1 1 1 1 -1 1]'};
%test_cases{end + 1} = {[-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3]', [0.1;0.1;0.1;0.1;0.1;0.1;0.1]};
case_desc = 'folding up'; case_fname = 'CfoldingUp'; test_cases{end + 1} = {deg2rad(joints(5,:))', [-4.87125941617731 -0.0735561342818407 0.63521184011302 4.18870325095684 1.91282383432598 3.3090539278487 -2.35266570423377]'}; % Starts at a singularity
%test_cases{end + 1} = {deg2rad(joints(12,:))', [3.60426218545767 1.26563657105162 -3.19483613264846 -6.92175875711803 -1.79462324164591 0.672289765429706 2.96056445057525]'};
%test_cases{end + 1} = {deg2rad(joints(12,:))', deg2rad(joints(12,:))' + randn(size(joints,2), 1) * 1 * pi};
%test_cases{end + 1} = {deg2rad(joints(12,:))' + randn(size(joints,2), 1) * 1 * pi, deg2rad(joints(12,:))' + randn(size(joints,2), 1) * 1 * pi};

lr = 0.1;
%method_name = 'J Inverse'; method_fname = 'JInv'; step_function = @(ang, tw) J_inverse_kinematics_step(robot, ang, tw, lr);
rrk0 = '1e-7'; method_name = sprintf('Redundancy Resolution (%s)', rrk0); method_fname = sprintf('RedRes%s', rrk0); step_function = @(ang, tw) redundancy_resolution_inverse_kinematics_step(robot, ang, tw, lr, str2double(rrk0), 0.01);
%method_name = 'DLS'; method_fname = 'DLS'; step_function = @(ang, tw) DLS_inverse_kinematics_step(robot, ang, tw, lr, .1);
%method_name = 'J transpose'; method_fname = 'JTrans'; step_function = @(ang, tw) J_transpose_inverse_kinematics_step(robot, ang, tw, lr);
plot_subtitle = sprintf('%s, lr=%.3f, uncapped', case_desc, lr); %%%capped \\pi/8', case_desc, lr);
video_fname = sprintf('video_%s_%s_uncapped_lr%4.2f.mp4', method_fname, case_fname, lr);

for i_test_cases = 1:numel(test_cases)
    %i = test_cases(i_test_cases);
    joint_angles = test_cases{i_test_cases}{1};
    % Find starting pose.
    start_pose = FK_space(robot, joint_angles);
    % Invent a destination pose.
    target_joint_angles = test_cases{i_test_cases}{2};
    target_pose = FK_space(robot, target_joint_angles);
    %step_function = @(ang, tw) step_function(ang, tw, target_pose);
    init_guess = joint_angles;
    fprintf('\nTwist from %s to %s\nJoints from %s to (for example) %s\n', mat2str(trans2twist(start_pose)', 5), mat2str(trans2twist(target_pose)', 5), mat2str(init_guess', 5), mat2str(target_joint_angles', 5));
    % Now solve it.
    [ik_angles, iter_errang, iter_errlin, iter_cond, iter_step, iter_stepnorm] = inverse_kinematics_movie(robot, init_guess, target_pose, step_function, method_name, plot_subtitle, video_fname, lr);
    ik_pose = FK_space(robot, ik_angles);
    % Print results
    fprintf('Max joint velocity: %.2f %s\n', max(max(abs(iter_step))) ./ lr, mat2str(max(abs(iter_step)) ./ lr, 3));
    fprintf('Joint Angle Solution: %s\n', mat2str(ik_angles', 5));
    fprintf('Angular error in ZYZ: %s\n', mat2str((rot2zyz(trans2rot(target_pose))-rot2zyz(trans2rot(ik_pose)))', 5));
    fprintf('Linear error in xyz: %s\n', mat2str((trans2translation(target_pose)-trans2translation(ik_pose))', 5));
    assert(all(rot2zyz(trans2rot(target_pose))-rot2zyz(trans2rot(ik_pose)) < 1e-1, 'all') || all(rot2rpy(trans2rot(target_pose))-rot2rpy(trans2rot(ik_pose)) < 1e-1, 'all'));
    assert(all((trans2translation(target_pose)-trans2translation(ik_pose)) < 1e-0, 'all'));
    
%     figure;
%     subplot(2, 1, 1);
%     title('Pose Error'); xlabel('Iteration'); hold all; yyaxis left; plot(0:numel(iter_errang)-1, iter_errang); ylabel('Norm of Angular Deviation'); yyaxis right; plot(0:numel(iter_errlin)-1, iter_errlin); ylabel('Norm of Linear Deviation');
%     subplot(2, 1, 2);
%     title('Condition and Step Size'); xlabel('Iteration'); hold all; yyaxis left; plot(0:numel(iter_cond)-1, iter_cond); ylabel('Condition Number'); yyaxis right; plot(0:numel(iter_stepnorm)-1, iter_stepnorm); ylabel('Norm Of Step');
end