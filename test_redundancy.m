
clear;
robot = robot_iiwa();
test_kuka_examples;

%% Explore IK examples
% Each example has a starting guess and a target, in angles (so we know are possible).
% The IK is run to get the cartesian pose of the target angles.
% Result is successful if pose from IK is very close to target pose.

test_cases = {};
%test_cases{end + 1} = {[0 .1 0 .1 0 0 0]', [0 0 0 0 0 0 0]'}; % Start non-singularity, move to singularity.
% THIS CASE STARTS AT SINGULARITY test_cases{end + 1} = {[0 0 0 0 0 0 0]', [0 -.1 .1 -.1 0 0 0]'}; % Start at singularity, move away.
%test_cases{end + 1} = {[0 .1 0 .1 0 0 0]', [0 -.1 0 -.1 0 0 0]'}; % Direct path is through singularity.
%test_cases{end + 1} = {[0 .1 .1 .1 0 0 0]', [0 -.1 .1 -.1 0 0 0]'};
%test_cases{end + 1} = {[-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3]', [0.1;0.1;0.1;0.1;0.1;0.1;0.1]};
% THIS CASE STARTS AT SINGULARITY test_cases{end + 1} = {deg2rad(joints(5,:))', [-4.87125941617731 -0.0735561342818407 0.63521184011302 4.18870325095684 1.91282383432598 3.3090539278487 -2.35266570423377]'};
test_cases{end + 1} = {deg2rad(joints(12,:))', [3.60426218545767 1.26563657105162 -3.19483613264846 -6.92175875711803 -1.79462324164591 0.672289765429706 2.96056445057525]'};
%test_cases{end + 1} = {deg2rad(joints(12,:))', deg2rad(joints(12,:))' + randn(size(joints,2), 1) * 1 * pi};

for i_test_cases = 1:numel(test_cases)
    %i = test_cases(i_test_cases);
    joint_angles = test_cases{i_test_cases}{1};
    % Find starting pose.
    start_pose = FK_space(robot, joint_angles);
    % Invent a destination pose.
    target_joint_angles = test_cases{i_test_cases}{2};
    target_pose = FK_space(robot, target_joint_angles);
    init_guess = joint_angles;
    fprintf('Twist from %s to %s\nJoints from %s to (for example) %s\n', mat2str(trans2twist(start_pose)', 5), mat2str(trans2twist(target_pose)', 5), mat2str(init_guess', 5), mat2str(target_joint_angles', 5));
    % Now solve it.
    [ik_angles, iter_errang, iter_errlin, iter_cond, iter_stepnorm] = redundancy_resolution(robot, init_guess, target_pose);
    ik_pose = FK_space(robot, ik_angles);
    % Print results
    fprintf('Joint Angle Solution: %s\n', mat2str(ik_angles', 5));
    fprintf('Angular error in ZYZ: %s\n', mat2str((rot2zyz(trans2rot(target_pose))-rot2zyz(trans2rot(ik_pose)))', 5));
    fprintf('Linear error in xyz: %s\n', mat2str((trans2translation(target_pose)-trans2translation(ik_pose))', 5));
    assert(all(rot2zyz(trans2rot(target_pose))-rot2zyz(trans2rot(ik_pose)) < 1e-2, 'all'));
    assert(all((trans2translation(target_pose)-trans2translation(ik_pose)) < 1e-2, 'all'));
    figure;
    subplot(2, 1, 1);
    title('Pose Error'); xlabel('Iteration'); hold all; yyaxis left; plot(0:numel(iter_errang)-1, iter_errang); ylabel('Norm of Angular Deviation'); yyaxis right; plot(0:numel(iter_errlin)-1, iter_errlin); ylabel('Norm of Linear Deviation');
    subplot(2, 1, 2);
    title('Condition and Step Size'); xlabel('Iteration'); hold all; yyaxis left; plot(0:numel(iter_cond)-1, iter_cond); ylabel('Condition Number'); yyaxis right; plot(0:numel(iter_stepnorm)-1, iter_stepnorm); ylabel('Norm Of Step');
end
%%
init_guess = [0.1;0.1;0.1;0.1;0.1;0.1;0.1];
redundancy_resolution(robot, init_guess, end_fram);