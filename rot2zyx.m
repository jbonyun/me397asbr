function zyx = rot2zyx(rot, delta)
    % Converts a rotation matrix to Euler ZYX
    % Inputs:
    %   rot: 3x3 rotation matrix in SO(3)
    %   delta: flag 1=pitch in [-pi/2,pi/2], -1=pitch in [pi/2, 3*pi/2]
    % Outputs:
    %   zyx: 3x1 vector of [yaw;pitch;roll] angles in radians
    % Jeff Bonyun (jb79332), jbonyun@utexas.edu, 20220323
    % On behalf of the Sun/Bonyun team for ME397 ASBR, Spring 2022.
    % Based on reversing the rot2zyx function we had previously written
    % using notes from Dr. Alambeigi.
    % The ZYX order is relevant, because it is the "A,B,C" that Kuka uses
    % to represent the orientation of their robots.
    
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
    % Assemble in yaw-pitch-roll order for our answer.
    zyx = [alpha; beta; gamma];

    % Verify that the other elements of the rotation matrix fit these
    % extracted r-p-y angles.
    recon = [cos(alpha)*cos(beta)    cos(alpha)*sin(beta)*sin(gamma)-sin(alpha)*cos(gamma)  cos(alpha)*sin(beta)*cos(gamma)+sin(alpha)*sin(gamma); 
             sin(alpha)*cos(beta)    sin(alpha)*sin(beta)*sin(gamma)+cos(alpha)*cos(gamma)  sin(alpha)*sin(beta)*cos(gamma)-cos(alpha)*sin(gamma);
             -sin(beta)              cos(beta)*sin(gamma)                                   cos(beta)*cos(gamma)                                 ];
    assert(all(abs(rot - recon) < 1e-9, 'all'));