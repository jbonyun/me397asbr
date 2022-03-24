function plot_3d_axis_transform(T, varargin)
    % Plots a red-green-blue, x-y-z axis in 3d figure.
    % Inputs:
    %   T: 4x4 transformation matrix
    %      upper left 3x3 is the rotation matrix
    %      top 3 rows of right column is translation
    % Jeff Bonyun (jb79332), jbonyun@utexas.edu, 20220215
    % On behalf of the Sun/Bonyun team for ME397 ASBR, Spring 2022.

    plot_3d_axis(T(1:3,4), T(1:3,1), T(1:3,2), T(1:3,3), varargin{:});
