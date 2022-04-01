function ellipsoid_plot_linear(J, varargin)
    % Plots manipulability ellipsoid of jacobian's linear portion.
    % Inputs:
    %   J: nxdof jacobian matrix
    % Outputs: none
    % Jeff Bonyun (jb79332), jbonyun@utexas.edu, 20220324
    % On behalf of the Sun/Bonyun team for ME397 ASBR, Spring 2022.

    assert(size(J,1) == 6, 'Jacobian must be in R6');

    % Take linear/translation portion of Jacobian.
    Jlin = J(1:3, :);

    plot_manipulability_ellipsoid(Jlin, varargin{:});
    title('Linear Manipulabity Ellipsoid');
    xlabel('x');
    ylabel('y');
    zlabel('z');
