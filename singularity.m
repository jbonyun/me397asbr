function singularity=singularity(robot,joint_angles)

% input robot and configurations
% output bool value 
    
    J(:, 1) = robot.screw(:, 1);
    prod_expon = expm_sym(robot.screw(:, 1) , joint_angles(1));
    for i = 2:robot.dof
    ad=adjoint_sym(prod_expon);
    J(:, i) = ad * robot.screw(:, i);
    prod_expon = prod_expon * expm_sym(robot.screw(:, i) , joint_angles(i));
    end
    if (rank(J)<6)
        singularity=ture;
    else 
        singularity=false;
    end