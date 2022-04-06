function [mu1] = J_isotropy(J)
    % Calculates the isotropy measure of singularity.
    % Inputs:
    %   J: nxdof jacobian matrix
    % Outputs:
    %   mu1: scalar isotropy of jacobian
    % Jeff Bonyun (jb79332), jbonyun@utexas.edu, 20220324
    % On behalf of the Sun/Bonyun team for ME397 ASBR, Spring 2022.
    % Source: Alambeigi, F. ASBR Lecture Notes. 2022, W8-L1, p 5.

    % Make sure the J matrix makes sense
    assert(mod(size(J,1),2) == 0, 'J matrix should have even number of rows');
    % Find the A matrix
    A = J * J';
    % Find eigenvalues of A
    eval = eig(A);
    % Special handling if singular.
    if min(eval) < 0
        if min(eval) < -1e-9
            % Negative definite. Not expecting this. I guess an error.
            error('Jacobian was negative-definite, with negative eigenvalue: %g', min(eval));
        else
            % Probably just numerically unstable, but really a zero eval
            mu1 = inf;
        end
    else
        emax = max(eval);
        emin = min(eval);
        mu1 = sqrt(emax) ./ sqrt(emin);
    end
