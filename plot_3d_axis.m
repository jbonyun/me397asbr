function plot_3d_axis(origin, xvec, yvec, zvec, varargin)
    % Plots a red-green-blue, x-y-z axis in 3d figure.
    % Inputs:
    %   origin: 3x1 x,y,z location of axis origin in figure coordinates
    %   xvec: 3x1 direction of the axis +x direction in figure coordinates
    %   yvec: 3x1 direction of the axis +y direction in figure coordinates
    %   zvec: 3x1 direction of the axis +z direction in figure coordinates
    % Jeff Bonyun (jb79332), jbonyun@utexas.edu, 20220215
    % On behalf of the Sun/Bonyun team for ME397 ASBR, Spring 2022.

    p = inputParser;
    isVec3 = @(x) numel(x) == 3 && sum(size(x) == 1) == ndims(x)-1;
    isAxes = @(x) isa(x, 'matlab.graphics.axis.Axes');
    addRequired(p, 'origin', isVec3);
    addRequired(p, 'xvec', isVec3);
    addRequired(p, 'yvec', isVec3);
    addRequired(p, 'zvec', isVec3);
    addOptional(p, 'ax', gca, isAxes);
    addParameter(p, 'Label', nan, @ischar);
    addParameter(p, 'scale', 1., @isfloat);
    addParameter(p, 'originscale', 1., @isfloat);
    p.KeepUnmatched = true;
    parse(p, origin, xvec, yvec, zvec, varargin{:});
    args = p.Results;
    args.ForwardArgs = [fieldnames(p.Unmatched) struct2cell(p.Unmatched)]';

    was_hold = ishold();
    hold on;

    % Plot them separately so that we can control the color individually.
    quiver3(args.ax, origin(1) * args.originscale, args.origin(2) * args.originscale, args.origin(3) * args.originscale, args.xvec(1) * args.scale, args.xvec(2) * args.scale, args.xvec(3) * args.scale, 'r', 'LineWidth', 2, 'MaxHeadSize', 5, args.ForwardArgs{:});
    quiver3(args.ax, origin(1) * args.originscale, args.origin(2) * args.originscale, args.origin(3) * args.originscale, args.yvec(1) * args.scale, args.yvec(2) * args.scale, args.yvec(3) * args.scale, 'g', 'LineWidth', 2, 'MaxHeadSize', 5, args.ForwardArgs{:});
    quiver3(args.ax, origin(1) * args.originscale, args.origin(2) * args.originscale, args.origin(3) * args.originscale, args.zvec(1) * args.scale, args.zvec(2) * args.scale, args.zvec(3) * args.scale, 'b', 'LineWidth', 2, 'MaxHeadSize', 5, args.ForwardArgs{:});
    if ~isnan(args.Label)
        text(origin(1) * args.originscale, origin(2 * args.originscale), origin(3) * args.originscale, args.Label, 'Interpreter', 'latex');
    end

    if ~was_hold
        hold off;
    end
