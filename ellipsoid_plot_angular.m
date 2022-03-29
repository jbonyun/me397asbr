function ellipsoid_plot_angular(J)
    % Plots manipulability ellipsoid of jacobian's angular portion.
    % Inputs:
    %   J: nxdof jacobian matrix
    % Outputs: none
    % Jeff Bonyun (jb79332), jbonyun@utexas.edu, 20220324
    % On behalf of the Sun/Bonyun team for ME397 ASBR, Spring 2022.
    % Source: Alambeigi, F. ASBR Lecture Notes. 2022, W8-L1, p 5.

    assert(size(J,1) == 6, 'Jacobian must be in R6');

    % Take angular portion of Jacobian.
    Jang = J(4:6, :);
    A = Jang * Jang';
    [evec, eval] = eig(A);
    % Make eval a vector instead of diagonal matrix.
    eval = diag(eval);
    % Make sure it is PD
    assert(all(min(eval) > 0), 'J*JT matrix is not positive-definite');

    % Make the ellipse shape, using sqrt(evals) as radius in each dim.
    [x,y,z] = ellipsoid(0, 0, 0, sqrt(eval(1)), sqrt(eval(2)), sqrt(eval(3)), 100);
    % TODO: rotate the ellipse by the eigenvectors!

    % Plot it!
    figure;
    surf(x, y, z);
    axis equal;
    title('Angular Manipulabity Ellipsoid');