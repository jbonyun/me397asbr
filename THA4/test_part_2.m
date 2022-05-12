%%

clc;
addpath('..');

%%% Prepare a robot struct
robot_no_tool = robot_iiwa();

start_angles=[0.2;0.2;0.2;0.2;0.2;0.2;0.2];
start_Ts = FK_space(robot_no_tool, start_angles);
start_pose=trans2translation(start_Ts);
p_goal=[127;45;1392];
target_Ts=[eye(3,3),p_goal; 0,0,0,1];
Z=[0;0;100];
Ptip=[eye(3),Z;0 0 0 1];
q=start_angles;
p_tip = trans2translation(start_Ts*Ptip);
Dis=norm(p_tip-p_goal)

while Dis> 0.1
 
 [dq] = Ls_opt(robot_no_tool, q,p_goal,Z)

 q=q+dq;
 tip_TS=FK_space(robot_no_tool,q);
 p_tip = trans2translation(tip_TS*Ptip);
 Dis=norm(p_tip-p_goal)
end
%%
   startTs = FK_space(robot_no_tool, start_angles);
    % Get t vector (translation of tip wrt space frame).
    Ptip=[eye(3),Z;0 0 0 1];
    t = trans2translation(startTs*Ptip);
    R= trans2rot(startTs);
    % Get vector pgoal from the destination location.
    
    Js = J_space(robot_no_tool, start_angles);
    Jalpha = Js(1:3, :);
    Jeps = Js(4:6, :);

    %qLb=[-0.1;-0.1;-0.1;-0.1;-0.1;-0.1;-0.1];
    %qUb=[0.1;0.1;0.1;0.1;0.1;0.1;0.1];
    qLb=[];
    qUb=[];
    
    delt=t-p_goal;
    
    
    
    max_distance =3;
    m =4; n = 4;  % Count of polygon mesh; bigger is a better approximation.
    [polyalpha,polybeta] = meshgrid(linspace(0, 2*pi * (1 - 1/m), m), linspace(0, 2*pi * (1 - 1/n), n));
    polyalpha = reshape(polyalpha, [], 1);
    polybeta = reshape(polybeta, [], 1);
    polyA = [cos(polyalpha).*cos(polybeta) cos(polyalpha).*sin(polybeta) sin(polyalpha)];
    polyb = repmat(max_distance, m*n, 1);
    A_sphere=polyA*(-skewsym(t) * Jalpha + Jeps);
    b_sphere=polyb-polyA *delt;
    qL = robot_no_tool.joint_limits(:, 1);
    qU = robot_no_tool.joint_limits(:, 2);
    
  dq =[-0.1128;-0.0174;-0.2521;-0.0078;-0.1654;0.0738;-0.2000];
  A_sphere*dq-b_sphere


 
