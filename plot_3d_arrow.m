function plot_3d_arrow(origin, direction, length, varargin)
    % Plots a red-green-blue, x-y-z axis in 3d figure.
    % Inputs:
    %   origin: 3x1 x,y,z location of axis origin in figure coordinates
    %   direction: 3x1 direction of the arrow from the origin
    %   length: scalar length of the arrow in direction
    % Jeff Bonyun (jb79332), jbonyun@utexas.edu, 20220215
    % On behalf of the Sun/Bonyun team for ME397 ASBR, Spring 2022.

    p = inputParser;
    isVec3 = @(x) numel(x) == 3 && sum(size(x) == 1) == ndims(x)-1;
    isAxes = @(x) isa(x, 'matlab.graphics.axis.Axes');
    addRequired(p, 'origin', isVec3);
    addRequired(p, 'direction', isVec3);
    addRequired(p, 'length', @isnumeric);
    addParameter(p, 'Axis', gca, isAxes);
    addParameter(p, 'Label', nan, @ischar);
    p.KeepUnmatched = true;
    parse(p, origin, direction, length, varargin{:});
    args = p.Results;
    args.ForwardArgs = [fieldnames(p.Unmatched) struct2cell(p.Unmatched)]';

    was_hold = ishold();
    hold on;

    quiver3(args.Axis, args.origin(1), args.origin(2), args.origin(3), direction(1)*args.length, direction(2)*args.length, direction(3)*args.length, args.ForwardArgs{:});
    if ~isnan(args.Label)
        text(origin(1), origin(2), origin(3), args.Label, 'Interpreter', 'latex');
    end

    if ~was_hold
        hold off;
    end
