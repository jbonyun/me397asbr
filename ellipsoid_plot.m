function ellipsoid_plot(J, varargin)
    % Plots manipulability ellipsoids of linear and angular sections.
    % Inputs:
    %   J: nxdof jacobian matrix
    % Outputs: none
    % Jeff Bonyun (jb79332), jbonyun@utexas.edu, 20220324
    % On behalf of the Sun/Bonyun team for ME397 ASBR, Spring 2022.

    assert(size(J,1) == 6, 'Jacobian must be in R6');

    figure;
    subplot(1, 2, 1);
    ellipsoid_plot_linear(J, 'Axis', gca);
    subplot(1, 2, 2);
    ellipsoid_plot_angular(J, 'Axis', gca);
