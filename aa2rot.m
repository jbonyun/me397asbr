function rot = aa2rot(aa)
    % Converts an axis/angle to rotation matrix.
    % Inputs:
    %   aa: 4x1, axis in 3x1 followed by angle in 4th index.
    %       angle must be in radians.
    % Outputs:
    %   rot: 3x3 rotation matrix in SO(3)
    % Jeff Bonyun (jb79332), jbonyun@utexas.edu, 20220209
    % On behalf of the Sun/Bonyun team for ME397 ASBR, Spring 2022.
    % Source: Alambeigi, F. ASBR Lecture Notes. 2022, W2-L2 p. 11. 

    % Transpose it if it was a row vector
    if all(size(aa) == [1 4])
        aa = aa';
    end

    % Requires 4x1 input
    assert(size(aa,1) == 4, 'aa2rot must have 4x1 input');
    assert(size(aa,2) == 1, 'aa2rot must have 4x1 input');

    % Split into axis and angle
    ax = aa(1:3,1);
    ang = aa(4);

    if all(ax == 0)
        error('Invalid axis of zero in all dimensions');
    end

    % If it isn't a unit axis vector, make it one.
    % Optionally, we could treat the length of the vector as a multiplier
    % on the angle. But I don't think I want that to be the default. So we
    % will just make it a unit vector and discard the length.
    if abs(norm(ax) - 1) > 1e-7
        disp('Warning: aa2rot given a non-unit axis and we are normalizing it, discarding magnitude');
        ax = ax / norm(ax);
    end
    
    % Convert to skew symmetric version of axis vector.
    axhat = skewsym(ax);

    rot = eye(3) + axhat * sin(ang) + axhat^2 * (1 - cos(ang));
