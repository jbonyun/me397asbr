
clear;
robot = robot_iiwa();
test_kuka_examples;

%% Test relationship between J_space and J_body

% From W7-L1, pg 13:
%   Jb = [Ad Tbs] Js
%   Js = [Ad Tsb] Jb
test_cases = [5];  %1:size(joints,1);
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
end

%% Test Jacobian function J_space

% TODO:
% Maybe finite differences with some velocities?
% Maybe a specific numerical example from somewhere?
% Maybe some thought-out theoretical examples?

%% Test Jacobian function J_body

% TODO: ditto above
