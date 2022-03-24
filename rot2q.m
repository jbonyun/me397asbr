function q = rot2q(rot)
    % Converts a rotation matrix to quaternion representation.
    % Inputs:
    %   rot: 3x3 rotation matrix in SO(3)
    % Outputs:
    %   q: 4x1 quaternion in w;x;y;z form (scalar first)
    % Jeff Bonyun (jb79332), jbonyun@utexas.edu, 20220209
    % On behalf of the Sun/Bonyun team for ME397 ASBR, Spring 2022.
    % Source: Alambeigi, F. ASBR Lecture Notes. 2022, W3-L1 p. 11.

    assert(isrot(rot), 'rot must be a valid rotation matrix');

    % Different sources order the elements of the quaternion differently.
    % We use w;x;y;z, which puts the real part first.

    % The built-in matlab `sign` function gives 0 for 0 input, and that
    % doesn't work well for us. So we make our own that outputs 1 for input
    % 0.
    sgn = @(x) (x >= 0) * 2 - 1;

    q = nan(4,1);
    q(1,1) = sqrt(rot(1,1) + rot(2,2) + rot(3,3) + 1) / 2;
    q(2,1) = sgn(rot(3,2) - rot(2,3)) * sqrt(rot(1,1) - rot(2,2) - rot(3,3) + 1) / 2;
    q(3,1) = sgn(rot(1,3) - rot(3,1)) * sqrt(rot(2,2) - rot(1,1) - rot(3,3) + 1) / 2;
    q(4,1) = sgn(rot(2,1) - rot(1,2)) * sqrt(rot(3,3) - rot(1,1) - rot(2,2) + 1) / 2;
