function ellipsoid_plot_angular(J, varargin)
    % Plots manipulability ellipsoid of jacobian's angular portion.
    % Inputs:
    %   J: nxdof jacobian matrix
    % Outputs: none
    % Jeff Bonyun (jb79332), jbonyun@utexas.edu, 20220324
    % On behalf of the Sun/Bonyun team for ME397 ASBR, Spring 2022.

    assert(size(J,1) == 6, 'Jacobian must be in R6');

    % Take linear/translation portion of Jacobian.
    Jang = J(4:6, :);

    plot_manipulability_ellipsoid(Jang, varargin{:});
    title('Angular Manipulabity Ellipsoid');
    xlabel('Z');
    ylabel('Y');
    zlabel('X');
