clear;

% Make sure you can use the functions in the parent directory.
addpath('..');

%%% Prepare a robot struct
robot_no_tool = robot_iiwa();
% Make a new robot struct with the tool added to the end effector
tool_length = 100;
robot_w_tool = robot_no_tool;  % Copies by value.
robot_w_tool.home(3,4) = robot_no_tool.home(3,4) + tool_length;  % Adds tool.
[robot_w_tool.screw, robot_w_tool.bscrew] = calc_screw(robot_w_tool.dof, robot_w_tool.offset, robot_w_tool.axes, robot_w_tool.home);

%start_angles = rand(robot_w_tool.dof, 1) .* (robot_w_tool.joint_limits(:,2) - robot_w_tool.joint_limits(:,1)) + robot_w_tool.joint_limits(:,1);
start_angles=[0.2;0.2;0.2;0.2;0.2;0.2;0.2];
start_Ts = FK_space(robot_w_tool, start_angles);
p_goal=[100;10;1100];
center=[126;18;1393];
Z=[0;0;100];
q=start_angles;
p_tip = trans2translation(start_Ts);
Dis=norm(p_tip-p_goal)
% Solve for the required change in joint angles to reach target.\
while Dis> 0.1
 
 dq=Ls_opt(robot_no_tool, start_angles, Z,p_goal,p_goal)
 q=q+dq;
 tip_TS=FK_space(robot_no_tool,q);
 p_tip = trans2translation(tip_TS);
 Dis=norm(p_tip-p_goal)
end


%%
clear
addpath('..');

%%% Prepare a robot struct
robot_no_tool = robot_iiwa();

start_angles=[0.2;0.2;0.2;0.2;0.2;0.2;0.2];
start_Ts = FK_space(robot_no_tool, start_angles);
p_goal=[100;10;1100];
target_Ts=[eye(3,3),p_goal; 0,0,0,1];
Z=[0;0;100];
Ptip=[eye(3),Z;0 0 0 1];
q=start_angles;
p_tip = trans2translation(start_Ts*Ptip);
Dis=norm(p_tip-p_goal)

while Dis> 0.1
 
 [dq] = Ls_opt2(robot_no_tool, q, target_Ts,Z)
 q=q+dq;
 tip_TS=FK_space(robot_no_tool,q);
 p_tip = trans2translation(tip_TS*Ptip);
 Dis=norm(p_tip-p_goal)
end


%%
clear
clc
%addpath('..');

%%% Prepare a robot struct
robot_no_tool = robot_iiwa();

start_angles=[0.2;0.2;0.2;0.2;0.2;0.2;0.2];
start_Ts = FK_space(robot_no_tool, start_angles);
p_goal=[100;10;1100];
target_Ts=[eye(3,3),p_goal; 0,0,0,1];
Z=[0;0;100];
Ptip=[eye(3),Z;0 0 0 1];
q=start_angles;
p_tip = trans2translation(start_Ts*Ptip);
Dis=norm(p_tip-p_goal)
normvector=[0 1 0];
P0=[100;10;1100];
d=15;
count=0;
while Dis>0.1
 
 [dq] = Ls_opt3(robot_no_tool, q, target_Ts,Z,normvector,P0,d)
 q=q+dq;
 tip_TS=FK_space(robot_no_tool,q);
 p_tip = trans2translation(tip_TS*Ptip);
 Dis=norm(p_tip-p_goal)
 count=count+1;
end


%%
clear
addpath('..');

%%% Prepare a robot struct
robot_no_tool = robot_iiwa();

start_angles=[0.2;0.2;0.2;0.2;0.2;0.2;0.2];
start_Ts = FK_space(robot_no_tool, start_angles);
p_goal=[100;10;1100];
target_Ts=[eye(3,3),p_goal; 0,0,0,1];
Z=[0;0;100];
Ptip=[eye(3),Z;0 0 0 1];
q=start_angles;
p_tip = trans2translation(start_Ts*Ptip);
Dis=norm(p_tip-p_goal)
normvector=[0 1 0];
P0=[100;10;1100];
d=15;
width=10;
hight=16;
count=0;
while Dis>0.1
 
 [dq] = Ls_opt4(robot_no_tool, start_angles, target_Ts,Z,normvector,P0,d,width,hight)
 q=q+dq;
 tip_TS=FK_space(robot_no_tool,q);
 p_tip = trans2translation(tip_TS*Ptip);
 Dis=norm(p_tip-p_goal)
 count=count+1;
end
