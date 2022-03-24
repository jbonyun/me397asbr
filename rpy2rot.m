function rot = rpy2rot(rpy)

    % Requires 3x1 input
    assert(size(rpy,1) == 3, 'Input rpy must be 3x1');
    assert(size(rpy,2) == 1, 'Input rpy must be 3x1');

    alpha = rpy(1); beta = rpy(2); gamma = rpy(3);
    rot = [cos(alpha)*cos(beta)    cos(alpha)*sin(beta)*sin(gamma)-sin(alpha)*cos(gamma)  cos(alpha)*sin(beta)*cos(gamma)+sin(alpha)*sin(gamma); 
           sin(alpha)*cos(beta)    sin(alpha)*sin(beta)*sin(gamma)+cos(alpha)*cos(gamma)  sin(alpha)*sin(beta)*cos(gamma)-cos(alpha)*sin(gamma);
             -sin(beta)              cos(beta)*sin(gamma)                                   cos(beta)*cos(gamma)                                 ];

    % Quality control.
    assert(isrot(rot));