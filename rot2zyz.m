function zyz = rot2zyz(rot, delta)
    % Converts an axis/angle to rotation matrix.
    % Inputs:
    %   rot: 3x3 rotation matrix in SO(3)
    %   delta: flag 1=Y in [0,pi], -1=Y in [-pi,0]
    % Outputs:
    %   zyz: 3x1 vector of [z;y;z] Euler angles.
    % Jeff Bonyun (jb79332), jbonyun@utexas.edu, 20220209
    % On behalf of the Sun/Bonyun team for ME397 ASBR, Spring 2022.
    % Source: Alambeigi, F. ASBR Lecture Notes. 2022, W2-L2 pp 16-17

    % Apply default value for delta of +1
    if nargin < 2
        delta = 1;
    end
    assert(delta == 1 || delta == -1, 'delta must be +1 or -1');

    assert(isrot(rot), 'rot must be a valid rotation matrix');

    if iseye(rot, 1e-5)
        disp('Warning: rot2zyz is choosing arbitrary Z rotation angles because rot is identity');
        zyz = [0 0 0]';
    else
        % Notes use phi, theta, psi
        % I am using alpha, beta, gamma to match textbook.
        alpha = atan2(delta * rot(2,3), delta * rot(1,3));
        beta = atan2(delta * sqrt(rot(1,3)^2 + rot(2,3)^2), rot(3,3));
        gamma = atan2(delta * rot(3,2), -delta * rot(3,1));
           
        if abs(mod(beta, pi)) < 1e-7
            error('ZYZ at singularity with beta=0 or pi');
        end

        % Should be in given ranges:
        assert(alpha >= -pi && alpha <= pi);
        assert(beta >= 0 && beta <= pi);
        assert(gamma >= -pi && gamma <= pi);
    
        % Verify answer is compatible with reconstructed matrix
        ca=cos(alpha); sa=sin(alpha); cb=cos(beta); sb=sin(beta); cg=cos(gamma); sg=sin(gamma);
        recon = [ca*cb*cg-sa*sg  -ca*cb*sg-sa*cg  ca*sb;
                 sa*cb*cg+ca*sg  -sa*cb*sg+ca*cg  sa*sb;
                 -sb*cg           sb*sg           cb];
        assert(all(abs(recon - rot) < 1e-9, 'all'), 'Rotation matrix does not reconstruct from ZYZ result');

        % Compose answer in ZYZ order
        zyz = [alpha; beta; gamma];
    end