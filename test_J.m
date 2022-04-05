
clear;
robot = robot_iiwa();
test_kuka_examples;

%% Test relationship between J_space and J_body

% From W7-L1, pg 13:
%   Jb = [Ad Tbs] Js
%   Js = [Ad Tsb] Jb
test_cases = [1];  %1:size(joints,1);
for i_test_cases = 1:numel(test_cases)
    i = test_cases(i_test_cases);
    joint_angles = joints(i,:);
    Js = J_space(robot, joint_angles);
    Jb = J_body(robot, joint_angles);
    Tsb = FK_space(robot, joint_angles);
    Tbs = FK_body(robot, joint_angles);

    diffs = Js - adjoint_transform(Tsb) * Jb;
    diffb = Jb - adjoint_transform(Tbs) * Js;
    errs = norm(diffs);
    errb = norm(diffb);
    assert(errs < 1e-7, 'Space is not transformed body');
    assert(errb < 1e-7, 'Body is not transformed space');
end

%% Test Jacobian J_space function against Matlab Robotics Toolbox
% Function robot.geometricJacobian should be comparable.
% It reports Jacobian for the given end effector name.

% TODO: THIS HAS NOT BEEN MADE TO WORK YET.

% Load the robot from the Matlab built-in set.
kuka = loadrobot('kukaIiwa14');
% Prepare an array of structs that can be used for the robot config
config = randomConfiguration(kuka);

eg_indices = [5];
for eg_i = 1:numel(eg_indices)
    eg_index = eg_indices(eg_i);
    joint_angles = deg2rad(joints(:, eg_index));
    ans = num2cell(joint_angles);
    [config.JointPosition] = ans{:};
    Jmatlab = kuka.geometricJacobian(config, 'iiwa_link_ee');
    Js = J_space(robot, joint_angles);
end

%% Test Jacobian function J_space via finite differences
% Since the (geometric) Jacobian is partial derivative of the angle and
% translation of the end effector for a change in a joint's angle,
% we can change the joint angles slightly, and see if the end effector
% frame changes by the jacobian.

% TODO: THIS IS STILL NOT WORKING.

delta = 0.01;
deltas = eye(robot.dof) * delta;
test_cases = [5];
for i_test_cases = 1:numel(test_cases)
    i = test_cases(i_test_cases);
    joint_angles = deg2rad(joints(i,:)');
    pose_init = FK_space(robot, joint_angles);
    J = J_space(robot, joint_angles);
    for j = 1:robot.dof
        pose = FK_space(robot, joint_angles + deltas(:, j));
        % If it was analytical jacobian, I think this would be right:
        %dpose = (trans2screw(pose) - trans2screw(pose_init)) / delta;
        % Change from one frame to another
        dpose = inv(pose_init) * pose;
        dposescrew = trans2screw(dpose, delta);
        err = J(:, j) - dposescrew;
        fprintf('Config %02d   Axis %1d  Linear err %f\n', i, j, norm(err(1:3)));
    end
end

%% Test Jacobian function J_space

% TODO:
% Maybe finite differences with some velocities?
% Maybe a specific numerical example from somewhere?
% Maybe some thought-out theoretical examples?

%% Test Jacobian function J_body

% TODO: ditto above
