function [screw, theta] = trans2screw(trans)
    % Converts a screw vector and a rotation angle around it
    % into a homogenous transformation matrix.
    % Inputs:
    %   trans: homogenous transformation matrix, 4x4
    % Outputs:
    %   screw: screw vector, 6x1
    %   theta: amount to move around screw, scalar    
    % Jeff Bonyun (jb79332), jbonyun@utexas.edu, 20220215
    % On behalf of the Sun/Bonyun team for ME397 ASBR, Spring 2022.
    % Source: notes taken during ASBR lecture W5-L1.

    assert(all(trans(4,:) == [0 0 0 1]), 'Not given a valid homogenous transformation matrix because last row is wrong');
    R = trans(1:3, 1:3);
    p = trans(1:3, 4);
    assert(isrot(R), 'Not given a valid homogenous transformation matrix because rotation is not a valid rotation matrix');

    if iseye(R)
        theta = norm(p);
        w = [0; 0; 0];
        v = p / theta;
    else
        wandtheta = rot2aa(R);
        w = wandtheta(1:3);
        theta = wandtheta(4);
        Ginv = @(w,theta) 1/theta * eye(3) - skewsym(w)/2 + (1/theta - cot(theta/2)/2) * skewsym(w)^2;
        v = Ginv(w, theta) * p;
    end

    screw = [w; v];