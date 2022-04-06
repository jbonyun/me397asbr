
function [mu2] = J_condition(J)
    % Calculates the condition number of jacobian
    % Inputs:
    %   J: nxdof jacobian matrix
    % Outputs:
    %   mu2: scalar condition number of jacobian
    % Jeff Bonyun (jb79332), jbonyun@utexas.edu, 20220324
    % On behalf of the Sun/Bonyun team for ME397 ASBR, Spring 2022.
    % Source: Alambeigi, F. ASBR Lecture Notes. 2022, W8-L1, p 5.

    % Make sure the J matrix makes sense
    assert(mod(size(J,1),2) == 0, 'J matrix should have even number of rows');
    if rank(J) < 6
        % Singular. Report exact values for singularity.
        mu2 = inf;
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
                mu2 = inf;
            end
        else
            emax = max(eval);
            emin = min(eval);
            mu2 = emax ./ emin;
        end
    end
