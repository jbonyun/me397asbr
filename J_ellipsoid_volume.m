function [mu3] = J_ellipsoid_volume(J)
    % Calculates the manipulabiity metric, which is the ellipsoid volume of
    % the jacobian.
    % Inputs:
    %   J: nxdof jacobian matrix
    % Outputs:
    %   mu3: scalar volume of manipulability ellipsoid.
    % Jeff Bonyun (jb79332), jbonyun@utexas.edu, 20220324
    % On behalf of the Sun/Bonyun team for ME397 ASBR, Spring 2022.
    % Source: Alambeigi, F. ASBR Lecture Notes. 2022, W8-L1, p 5.

    % Make sure the J matrix makes sense
    % We assume it is half translation and half orientation, like our
    % typical 6-degree world.
    assert(mod(size(J,1),2) == 0, 'J matrix should have even number of rows');
    if rank(J) < 6
        % Singular. Report exact values for singularity.
        mu3 = 0;
    else
        % Find the A matrix
        A = J * J';
        % Find eigenvalues of A
        eval = eig(A);
        % Special handling if singular.
        if min(eval) < 0
            if min(eval) < -1e-9
                % Negative definite. Not expecting this. I guess an error.
                error('Jacobian was negative-definite, with negative eigenvalue');
            else
                % Probably just numerically unstable, but really a zero eval
                mu3 = 0;
            end
        else
            mu3 = sqrt(prod(eval));
        end
    end
