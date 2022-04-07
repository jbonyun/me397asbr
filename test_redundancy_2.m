
clear;
robot = robot_iiwa();
test_kuka_examples;

%% Explore IK examples
% Each example has a starting guess and a target, in angles (so we know are possible).
% The IK is run to get the cartesian pose of the target angles.
% Result is successful if pose from IK is very close to target pose.

test_cases = {};
%test_cases{end + 1} = {deg2rad(joints(5,:))', [-4.87125941617731 -0.0735561342818407 0.63521184011302 4.18870325095684 1.91282383432598 3.3090539278487 -2.35266570423377]'};
% test_cases{end + 1} = {deg2rad(joints(12,:))', [3.60426218545767 1.26563657105162 -3.19483613264846 -6.92175875711803 -1.79462324164591 0.672289765429706 2.96056445057525]'};
% test_cases{end + 1} = {deg2rad(joints(12,:))', deg2rad(joints(12,:))' + randn(size(joints,2), 1) * 1 * pi};
% test_cases{end + 1} = {[0 .1 .1 .1 0 0 0]', [0 -.1 .1 -.1 0 0 0]'};
test_cases{end + 1} = {[-0.3,-0.3,-0.3,-0.3,-0.3,-0.3,-0.3]', [0.1;0.1;0.1;0.1;0.1;0.1;0.1]};
% test_cases{end + 1} = {[0 .1 0 .1 0 0 0]', [0 -.1 0 -.1 0 0 0]'}; % Direct path is through singularity.
% test_cases{end + 1} = {[0 .1 0 .1 0 0 0]', [0 0 0 0 0 0 0]'}; % Start off singularity, move to singularity.
% test_cases{end + 1} = {[0 0 0 0 0 0 0]', [0 -.1 .1 -.1 0 0 0]'}; % Start at singularity, move away.
for i_test_cases = 1:numel(test_cases)
    %i = test_cases(i_test_cases);
    joint_angles = test_cases{i_test_cases}{1};
    % Find starting pose.
    start_pose = FK_space(robot, joint_angles);
    % Invent a destination pose.
    target_joint_angles = test_cases{i_test_cases}{2};
    %target_joint_angles = joint_angles + randn(size(joint_angles)) * 1 * pi;
    target_pose = FK_space(robot, target_joint_angles);
    init_guess = joint_angles;
    % Now solve it.
    ik_angles = redundancy_resolution_2(robot, init_guess, target_pose);
    ik_pose = FK_space(robot, ik_angles);
    angle_compare = [joint_angles target_joint_angles ik_angles]
    cond_compare = [J_condition(J_space(robot, joint_angles)) J_condition(J_space(robot, target_joint_angles)) J_condition(J_space(robot, ik_angles))]
    zyz_compare_and_diff = [rot2zyz(trans2rot(target_pose)) rot2zyz(trans2rot(ik_pose)) rot2zyz(trans2rot(target_pose))-rot2zyz(trans2rot(ik_pose))]
    lin_compare_and_diff = [trans2translation(target_pose) trans2translation(ik_pose) trans2translation(target_pose)-trans2translation(ik_pose)]
    assert(all(zyz_compare_and_diff(:,end) < 1e-2, 'all'));
    assert(all(lin_compare_and_diff(:,end) < 1e-2, 'all'));
end

