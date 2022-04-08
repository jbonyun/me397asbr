function [twist] = trans2twist(trans)
    % Converts a transformation matrix into a screw vector that summarizes
    % the linear and angular elements in a 6x1 vector.
    % Inputs:
    %   trans: homogenous transformation matrix, 4x4
    % Outputs:
    %   twist: twist vector, 6x1
    % Jeff Bonyun (jb79332), jbonyun@utexas.edu, 20220409
    % On behalf of the Sun/Bonyun team for ME397 ASBR, Spring 2022.

    [screw, theta] = trans2screw(trans);
    twist = screw2twist(screw, theta);
