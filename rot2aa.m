function aa = rot2aa(rot)
    % Converts a rotation matrix to axis-angle representation.
    % Inputs:
    %   rot: 3x3 rotation matrix in SO(3)
    % Outputs:
    %   aa: 4x1 vector where first 3 are rotation axis, 4th is angle in rad
    % Jeff Bonyun (jb79332), jbonyun@utexas.edu, 20220209
    % On behalf of the Sun/Bonyun team for ME397 ASBR, Spring 2022.
    % Source: Alambeigi, F. ASBR Lecture Notes. 2022, W2-L2 pp. 8-11.
    
    assert(isrot(rot), 'rot must be a valid rotation matrix');

    % Check for being close to identity matrix.
    if iseye(rot, 1e-5)
        % When we are rotating by identity the axis doesn't matter and
        % angle about it is 0. But we keep axis a unit vector anyway.
        % Strictly speaking, he says the axis is undefined. I don't think
        % we need to throw an error though, as it also seems valid to just
        % choose an arbitrary axis to not spin about.
        theta = 0;
        w = [1; 0; 0;];
    elseif abs(trace(rot) - -1) < 1e-10
        theta = pi; 
        if abs(rot(1,1) - -1) > 1e-10
            w = [rot(1,1)+1; rot(2,1); rot(3,1)] ./ sqrt(2 * (1 + rot(1,1)));
        elseif abs(rot(2,2) - -1) > 1e-10
            w = [rot(1,2); rot(2,2)+1; rot(3,2)] ./ sqrt(2 * (1 + rot(2,2)));
        elseif abs(rot(3,3) - -1) > 1e-10
            w = [rot(1,3); rot(2,3); 1+rot(3,3)] ./ sqrt(2 * (1 + rot(3,3)));
        else
            error('Something is wrong with your rotation matrix. Impossible!');
        end
    else
        theta = acos((trace(rot) - 1)/ 2);
        what = (rot - rot') / 2 / sin(theta);
        w = unskewsym(what);
    end

    % Make sure axis is a unit vector
    assert(abs(norm(w) - 1) < 1e-5, sprintf('Axis is not a unit vector with norm %f', norm(w)));

    % Combine axis and angle into a single output vector of 4x1
    aa = [w; theta];    
