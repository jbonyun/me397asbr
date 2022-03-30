function plot_joint_axis(joint_frame, joint_axis_at_home, length, varargin)
    % Plots a red-green-blue, x-y-z axis in 3d figure.
    % Inputs:
    %   joint_frame: 4x4 transformation matrix for the joint's frame
    %   joint_axis_at_home: a 3x1 axis of the joint when in home position
    %   length: scalar length of the arrow in direction
    % Jeff Bonyun (jb79332), jbonyun@utexas.edu, 20220215
    % On behalf of the Sun/Bonyun team for ME397 ASBR, Spring 2022.

    p = inputParser;
    addRequired(p, 'joint_frame', @(x)(validateattributes(x, {'numeric'}, ...
                {'nonempty', 'real', 'nonnan', 'finite', '2d', 'size', [4 4]})));
    addRequired(p, 'joint_axis_at_home', @(x)(validateattributes(x, {'numeric'}, ...
                {'nonempty', 'real', 'nonnan', 'finite', 'vector', 'numel', 3})));
    addRequired(p, 'length', @isnumeric);
    p.KeepUnmatched = true;
    parse(p, joint_frame, joint_axis_at_home, length, varargin{:});
    args = p.Results;
    args.ForwardArgs = [fieldnames(p.Unmatched) struct2cell(p.Unmatched)]';

    root = args.joint_frame * [-length * args.joint_axis_at_home; 1];
    root = root(1:3);
    dir = args.joint_frame * [args.joint_axis_at_home; 1] - joint_frame(:,4);
    dir = dir(1:3);
    plot_3d_arrow(root, dir, length*2, 'LineWidth', 1, 'LineStyle', ':', 'Color', 'k');
