function J_dagger=dagger_J(jacobian,n,m)
    if n>m
        J_dagger=transpose(jacobian)*inv(jacobian*transpose(jacobian));
    elseif n<m
        J_dagger=inv(transpose(jacobian)*jacobian)*transpose(jacobian);
    end
    


    

