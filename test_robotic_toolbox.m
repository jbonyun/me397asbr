clear
robot = importrobot('iiwa14.urdf');
%joint_angles=[0;0;0;pi/2;pi/4;0;0]
%joint_angles=[0;pi/2;pi/4;0;0;0;0];
%joint_angles=[0;pi/4;0;pi/2;pi/2;0;0];
%joint_angles=[0;pi/2;pi;0;0;-pi/2;0];
%joint_angles=[pi/2;0;-pi/2;pi/2;0;pi/2;0];
joint_angles=[0.5;0.5;0.5;0.5;0.5;0.5;0.5];
%config = randomConfiguration(robot)
config = homeConfiguration(robot);
for i=1:7
    config(i).JointPosition = joint_angles(i);
end
tform = getTransform(robot,config,'iiwa_link_ee_kuka','iiwa_link_0');
jacobian = geometricJacobian(robot,config,'iiwa_link_7')
show(robot,config)
rank(jacobian)
%showdetails(robot)

