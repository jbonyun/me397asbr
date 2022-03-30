% Graphical demonstration of the robot

% Load the robot from the Matlab built-in set.
kuka = loadrobot('kukaIiwa14');
% Prepare an array of structs that can be used for the robot config
config = randomConfiguration(kuka);
% Replace the random joint angles with my own
example_pose = joints(6, :);
num2cell(deg2rad(example_pose));
[config.JointPosition] = ans{:};
% Display the resulting configuration.
show(kuka, config);