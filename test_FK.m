clear;

% Load robot info
robot = robot_iiwa();

% Load example poses for Kuka robot
test_kuka_examples;

%% Test FK_space
% To see that our robot description and FK_space match what Kuka
% software reports. We want our model to match the Kuka software as close
% as possible.

% Run through our examples, so see if we calculate the same cartesian
% position for the given joint positions.
do_plot = true;
i_set = [5]; %1:16;
for i = 1:numel(i_set)
    fk_pose = FK_space(robot, deg2rad(joints(i_set(i),:)'), 'DoPlot', do_plot);
    if do_plot
        title(sprintf('Joint Frames\nConfig %d', i_set(i)));
    end
    err = norm(fk_pose(1:3, 4)-cart(i_set(i), 1:3)');
    angerr = norm(rad2deg(rot2zyx(fk_pose(1:3,1:3)))-cart(i_set(i), 4:6)');
    fprintf("Space Frame  Pose %2d   Pos error: %f   Angle error: %f\n", i_set(i), err, angerr);
    if err > 0.10
        disp([fk_pose(1:3, 4) cart(i_set(i), 1:3)']);
    end
    if angerr > 0.10
        fprintf('Angle not matching for %d\n', i);
        disp([rad2deg(rot2zyx(fk_pose(1:3,1:3))) cart(i_set(i), 4:6)']);
    end
end

%% Test FK_body

i_set = 1:16;
for i = 1:numel(i_set)
    fk_pose = FK_body(robot, deg2rad(joints(i_set(i),:)'));
    if do_plot
        title(sprintf('Joint Frames; Body Form\nConfig %d', i_set(i)));
    end
    err = norm(fk_pose(1:3, 4)-cart(i_set(i), 1:3)');
    angerr = norm(rad2deg(rot2zyx(fk_pose(1:3,1:3)))-cart(i_set(i), 4:6)');
    fprintf("Body Frame  Pose %2d   Pos error: %f   Angle error: %f\n", i_set(i), err, angerr);
    if err > 0.10
        disp([fk_pose(1:3, 4) cart(i_set(i), 1:3)']);
    end
    if angerr > 0.10
        fprintf('Angle not matching for %d\n', i);
        disp([rad2deg(rot2zyx(fk_pose(1:3,1:3))) cart(i_set(i), 4:6)']);
    end
end

%% Test FK_space and FK_body give same answers
i_set = 1:16;
for i = 1:numel(i_set)
    fk_s = FK_space(robot, deg2rad(joints(i_set(i),:)'));
    fk_b = FK_body(robot, deg2rad(joints(i_set(i),:)'));
    err = norm(fk_s - fk_b);
    assert(err < 1e-12, 'FK_space and FK_body gave different answers');
end
disp('All good');