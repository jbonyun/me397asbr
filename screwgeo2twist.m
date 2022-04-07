function [twist] = screwgeo2twist(shat_axis, q_point, h_pitch, thetadot)
    assert(all(size(shat_axis) == [3 1]), 'w_axis must be 3x1');
    assert(all(size(q_point) == [3 1]), 'q_point must be 3x1');
    if nargin < 3
        h_pitch = 0;
    end
    if nargin < 4
        thetadot = 1;
    end
    v = -cross(shat_axis, q_point) + h_pitch * shat_axis;
    twist = [shat_axis; v] * thetadot;