function [dq] = Ls_opt4(robot_no_tool, start_angles, destTs,Z,normvector,P0,distance,width,hight)
%define an infinte plane
%normvector 1*3
%P0 point on the plane 3*1
%d scalar distance

    startTs = FK_space(robot_no_tool, start_angles);
    Ptip=[eye(3),Z;0 0 0 1];
    t = trans2translation(startTs*Ptip);
    R= trans2rot(startTs);
    % Get vector pgoal from the destination location.
    pgoal = trans2translation(destTs);
    Js = J_space(robot_no_tool, start_angles);
    Jalpha = Js(1:3, :);
    Jeps = Js(4:6, :);
    Jb = J_body(robot_no_tool, start_angles);
    Jbalpha = Jb(1:3, :);
    Jbeps = Jb(4:6, :);


    qLb=[-0.1;-0.1;-0.1;-0.1;-0.1;-0.1;-0.1];
    qUb=[0.1;0.1;0.1;0.1;0.1;0.1;0.1];
    
    delt=t-pgoal;
    
    
    
    max_distance = 20000;
    m = 10; n = 10;  % Count of polygon mesh; bigger is a better approximation.
    [polyalpha,polybeta] = meshgrid(linspace(0, 2*pi * (1 - 1/m), m), linspace(0, 2*pi * (1 - 1/n), n));
    polyalpha = reshape(polyalpha, [], 1);
    polybeta = reshape(polybeta, [], 1);
    polyA = [cos(polyalpha).*cos(polybeta) cos(polyalpha).*sin(polybeta) sin(polyalpha)];
    polyb = repmat(max_distance, m*n, 1)-polyA*delt;
    polyA =polyA*Jeps;
    qL = robot_no_tool.joint_limits(:, 1);
    qU = robot_no_tool.joint_limits(:, 2);
    planA=-normvector*R*(-skewsym(Z)*Jbalpha+Jbeps);
    planb=normvector*t-normvector*P0-distance;
    
    a=1;
 
    C1= -skewsym(t) * Jalpha + Jeps;
    d1=pgoal - t; 


    C = a*C1;
    d = a*d1;
%     if (abs(t(1)-P0(1))<(width/2)) && (abs(t(3)-P0(3))<(hight/2))
%         A=[polyA;eye(7);-eye(7);planA];
%         b=[polyb;qU-start_angles;start_angles-qL;planb];
%     else
%          A=[polyA;eye(7);-eye(7)];
%          b=[polyb;qU-start_angles;start_angles-qL];
%     end
    A=[polyA;eye(7);-eye(7)];
    b=[polyb;qU-start_angles;start_angles-qL];
    Aeq=[];
    beq=[];
    dq=lsqlin(C,d,A,b,Aeq,beq,qLb,qUb);




