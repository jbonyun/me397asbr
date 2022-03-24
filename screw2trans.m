function transf = screw2trans(screw, theta)
    % Converts a screw vector and a rotation angle around it
    % into a homogenous transformation matrix.
    % Inputs:
    %   screw: screw vector, 6x1
    %   theta: amount to move around screw, scalar
    % Outputs:
    %   trans: homogenous transformation matrix, 4x4
    % Jeff Bonyun (jb79332), jbonyun@utexas.edu, 20220215
    % On behalf of the Sun/Bonyun team for ME397 ASBR, Spring 2022.
    % Source: note taken during ASBR lecture W5-L1.

    w = screw(1:3);
    v = screw(4:6);
    
    if abs(norm(w) - 1) < 1e-8
        % theta is the angle
        transf = [expm(skewsym(w) * theta) (eye(3)*theta + (1-cos(theta))*skewsym(w) + (theta - sin(theta))*skewsym(w)^2)*v ; 0 0 0 1];
    elseif abs(norm(w) - 0) < 1e-8
        assert(abs(norm(v) - 1) < 1e-8, 'One of w or v part of screw must be unit length');
        % theta is the distance
        transf = [eye(3) v*theta; 0 0 0 1];
    else
        error('Angular portion of screw is not normalized (0 or 1)');
    end
