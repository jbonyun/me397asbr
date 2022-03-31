function singularity=check_singularity(robot,config)
    
    jocobian=J_space(robot,config);
    if (rank(jocobian)<6)
        singularity=ture;
    else 
        singularity=false;
    end