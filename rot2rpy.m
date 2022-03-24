function rpy = rot2rpy(rot, delta)
    % Converts a rotation matrix to roll-pitch-yaw (which is Euler ZYX)
    % Inputs:
    %   rot: 3x3 rotation matrix in SO(3)
    %   delta: flag 1=pitch in [-pi/2,pi/2], -1=pitch in [pi/2, 3*pi/2]
    % Outputs:
    %   rpy: 3x1 vector of [roll;pitch;yaw] angles in radians
    % Jeff Bonyun (jb79332), jbonyun@utexas.edu, 20220209
    % On behalf of the Sun/Bonyun team for ME397 ASBR, Spring 2022.
    % Source: Alambeigi, F. ASBR Lecture Notes. 2022, W2-L2 p. 19. 
    
    % Apply default value for delta of +1
    if nargin < 2
        delta = 1;
    end
    assert(delta == 1 || delta == -1, 'delta must be +1 or -1');

    % Requires 3x3 input that is a rotation matrix.
    assert(isrot(rot), 'rot must be a valid rotation matrix');

    % He uses phi, theta, psi where I am using alpha, beta, gamma to match
    % textbook.
    beta = atan2(-rot(3,1), delta * sqrt(rot(3,2)^2 + rot(3,3)^2));
    if abs(abs(beta) - pi/2) < 1e-9
        error('Roll-Pitch-Yaw in singularity at pitch=pi/2');
    end
    alpha = atan2(delta * rot(2,1), delta * rot(1,1));
    gamma = atan2(delta * rot(3,2), delta * rot(3,3));
    % Assemble in roll-pitch-yaw order for our answer.
    rpy = [gamma; beta; alpha];

    % Verify that the other elements of the rotation matrix fit these
    % extracted r-p-y angles.
    recon = [cos(alpha)*cos(beta)    cos(alpha)*sin(beta)*sin(gamma)-sin(alpha)*cos(gamma)  cos(alpha)*sin(beta)*cos(gamma)+sin(alpha)*sin(gamma); 
             sin(alpha)*cos(beta)    sin(alpha)*sin(beta)*sin(gamma)+cos(alpha)*cos(gamma)  sin(alpha)*sin(beta)*cos(gamma)-cos(alpha)*sin(gamma);
             -sin(beta)              cos(beta)*sin(gamma)                                   cos(beta)*cos(gamma)                                 ];
    assert(all(abs(rot - recon) < 1e-9, 'all'));