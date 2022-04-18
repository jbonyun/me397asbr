function [trans] = twist2trans(tw)
    % Converts a twist vector into a screw vector. Which is a normalized
    % version of twist.
    % Inputs:
    %   tw: 6x1 twist vector
    % Outputs:
    %   trans: 4x4 translation matrix
    % Jeff Bonyun (jb79332), jbonyun@utexas.edu, 20220417
    % On behalf of the Sun/Bonyun team for ME397 ASBR, Spring 2022.

    [scrax, theta] = twist2screw(tw);
    trans = screw2trans(scrax, theta);
