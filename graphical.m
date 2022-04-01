% Graphical demonstration of the robot

clear;
test_kuka_examples;

% Load the robot from the Matlab built-in set.
kuka = loadrobot('kukaIiwa14');

% Prepare an array of structs that can be used for the robot config
config = randomConfiguration(kuka);

eg_poses = 1:16; %[1 2];
for i = 1:numel(eg_poses)
    % Replace the random joint angles with my own
    eg_pose_num = eg_poses(i);
    example_pose = joints(eg_pose_num, :);
    num2cell(deg2rad(example_pose));
    [config.JointPosition] = ans{:};
    % Display the resulting configuration.
    show(kuka, config);
    title(sprintf('Test Config %d', eg_pose_num));
    % Wait for input before continuing.
    pause;
end
close;
