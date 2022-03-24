function rot = q2rot(q)
    % Converts a quaternion into a rotation matrix
    % Inputs:
    %   q: 4x1 quaternion
    % Outputs:    
    %   rot: 3x3 rotation matrix in SO(3)
    % Jeff Bonyun (jb79332), jbonyun@utexas.edu, 20220210
    % On behalf of the Sun/Bonyun team for ME397 ASBR, Spring 2022.
    % Source: Lynch and Park. Modern Robotics, 2017, p. 581

    % Transpose it if it was a row vector
    if all(size(q) == [1 4])
        q = q';
    end

    % Requires 4x1 input
    assert(size(q,1) == 4, 'Input quaternion must be 4x1');
    assert(size(q,2) == 1, 'Input quaternion must be 4x1');

    if abs(norm(q) - 0) < 1e-7
        error('Quaternion has invalid zero length');
    elseif abs(norm(q) - 1) > 1e-7
        disp('Warning: you gave a non-unit quaternion and we are normalizing it');
        q = q / norm(q);
    end

    % Extract terms to match textbook and make shorter:
    q0 = q(1); q1 = q(2); q2 = q(3); q3 = q(4);
    rot = [q0^2+q1^2-q2^2-q3^2  2*(q1*q2-q0*q3)     2*(q0*q2+q1*q3);
           2*(q0*q3+q1*q2)      q0^2-q1^2+q2^2-q3^2 2*(q2*q3-q0*q1);
           2*(q1*q3-q0*q2)      2*(q0*q1+q2*q3)     q0^2-q1^2-q2^2+q3^2];

    % Quality control.
    assert(isrot(rot), 'q2rot result isnt a valid rotation matrix');
