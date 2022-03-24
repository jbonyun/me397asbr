function rot = zyz2rot(zyz)
    % Converts an axis/angle to rotation matrix.
    % Inputs:
    %   zyz: 3x1 vector of [z;y;z] euler angles.
    % Outputs:
    %   rot: 3x3 rotation matrix in SO(3)
    % Jeff Bonyun (jb79332), jbonyun@utexas.edu, 20220215
    % On behalf of the Sun/Bonyun team for ME397 ASBR, Spring 2022.
    % Source: Alambeigi, F. ASBR Lecture Notes. 2022, W2-L2 p. 16.

    % Note: this function was not required for assignment, but is included
    % for completeness.

    alpha = zyz(1); beta = zyz(2); gamma = zyz(3);
    ca=cos(alpha); sa=sin(alpha); cb=cos(beta); sb=sin(beta); cg=cos(gamma); sg=sin(gamma);
    rot = [ca*cb*cg-sa*sg  -ca*cb*sg-sa*cg  ca*sb;
           sa*cb*cg+ca*sg  -sa*cb*sg+ca*cg  sa*sb;
           -sb*cg           sb*sg           cb];

    % Quality control
    assert(isrot(rot));