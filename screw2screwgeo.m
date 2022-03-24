function [point, direction, h] = screw2screwgeo(screw, theta)
    % Converts a screw axis vector and a rotation angle around it
    % into the geometric interpretation of a screw.
    % Inputs:
    %   screw: screw vector, 6x1
    %   theta: amount to move around screw, scalar    
    % Outputs:
    %   point: the point on the screw axis that the screw passes through
    %  direct: the direction vector from the point along the axis
    %       h: the pitch of the screw
    % Jeff Bonyun (jb79332), jbonyun@utexas.edu, 20220215
    % On behalf of the Sun/Bonyun team for ME397 ASBR, Spring 2022.
    % Source: note taken during ASBR lecture W5-L1.

    w = screw(1:3);
    v = screw(4:6);

    shat = w;  % I think, always?

    % We know that v = -cross(shat, q) + h * shat
    % h is the only way to move in direction of shat
    % So all the translation in v in the direction of shat must come from h
    % And h is the ratio of translation/rotation. Since rotation is already
    % unit length in shat, it's just the length of projection of v on shat.
    h = dot(shat, v);
    % The vector of that projection is h along shat direction
    h_translation = h * shat;
    % And the remaining translation in v is all perpendicular to shat
    v_perp = v - h_translation;

    % Now we can have -cross(shat, q) = v_perp
    %                = [shat] * q = -v_perp

    % We can solve this as a system of linear equations.
    % However, q is actually any point along a line with direction shat.
    % So the matrix is singular and we need another constraint.
    % lsqminnorm adds that constraint, minimizing its distance from the
    % origin. Is this great? Meh. It works.
    q = lsqminnorm(skewsym(shat), -v_perp);

    point = q;
    direction = shat;