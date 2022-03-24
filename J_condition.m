function [mu2] = J_condition(J)
    % Calculates the condition number of jacobian
    % Inputs:
    %   J: nxdof jacobian matrix
    % Outputs:
    %   mu2: 1x2 condition numbers of translation and orientation in jacobian
    % Jeff Bonyun (jb79332), jbonyun@utexas.edu, 20220324
    % On behalf of the Sun/Bonyun team for ME397 ASBR, Spring 2022.
    % Source: Alambeigi, F. ASBR Lecture Notes. 2022, W8-L1, p 5.

    % Make sure the J matrix makes sense
    assert(mod(size(J,1),2) == 0, 'J matrix should have even number of rows');
    n = size(J,1) / 2;
    % Split it into translation and orientation
    Jtrans = J(1:n, :);
    Jorien = J((n+1):end, :);
    % Find the A matrix
    Atrans = Jtrans*Jtrans';
    Aorien = Jorien*Jorien';
    % Find eigenvalues of A
    eval = [eig(Atrans) eig(Aorien)];
    % Make sure it is PD
    assert(all(min(eval) > 0), 'J*JT matrix is not positive-definite');
    % Calculate
    emax = max(eval);
    emin = min(eval);
    mu2 = emax ./ emin;