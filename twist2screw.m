function [scr, thetadot] = twist2screw(tw)
    % Converts a twist vector into a screw vector. Which is a normalized
    % version of twist.
    % Inputs:
    %   tw: 6x1 twist vector
    % Outputs:
    %   scr: 6x1 screw vector
    % Jeff Bonyun (jb79332), jbonyun@utexas.edu, 20220215
    % On behalf of the Sun/Bonyun team for ME397 ASBR, Spring 2022.
    % Source: Modern Robotics. pp. 103-104.

    w = tw(1:3);
    v = tw(4:6);
    thetadot = norm(w);
    if abs(norm(w)) < 1e-9
        % If there is no rotation, we normalize by linear velocity instead.
        thetadot = norm(v);
    end
    scr = tw ./ thetadot;