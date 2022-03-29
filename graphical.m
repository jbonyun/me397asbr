% Graphical demonstration of the robot

% A demonstration of the basics
kuka = loadrobot('kukaIiwa14');
config = randomConfiguration(kuka);
show(kuka, config);