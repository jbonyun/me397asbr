function plot_screw(q, direction, h, theta, varargin)
    % Plots a screw axis in geometric interpretation.
    % For now, this is just the q and direction as an arrow.
    % Inputs:
    %   q: 3x1 origin point for the axis
    %   direction: 3x1 direction from the point q
    %   h: the pitch of the screw
    %   theta: the number of radians about the axis to rotate
    % Jeff Bonyun (jb79332), jbonyun@utexas.edu, 20220219
    % On behalf of the Sun/Bonyun team for ME397 ASBR, Spring 2022.

    p = inputParser;
    isVec3 = @(x) numel(x) == 3 && sum(size(x) == 1) == ndims(x)-1;
    isAxes = @(x) isa(x, 'matlab.graphics.axis.Axes');
    addRequired(p, 'q', isVec3);
    addRequired(p, 'direction', isVec3);
    addRequired(p, 'h', @isnumeric);
    addRequired(p, 'theta', @isnumeric);
    addOptional(p, 'ax', gca, isAxes);
    addParameter(p, 'LineSpec', 'r', @ischar);
    addParameter(p, 'Label', nan, @ischar);
    addParameter(p, 'LabelQ', true, @isbool);
    p.KeepUnmatched = true;
    parse(p, q, direction, h, theta, varargin{:});
    args = p.Results;
    args.ForwardArgs = [fieldnames(p.Unmatched) struct2cell(p.Unmatched)];

    was_hold = ishold();
    hold on;

    % q point
    plot3(q(1), q(2), q(3), 'k.', 'MarkerSize', 20);
    if args.LabelQ
        text(q(1), q(2), q(3), '$\enspace q$', 'Interpreter', 'Latex');
    end

    % Arrow from q along direction for h*theta.
    if h == 0
        line_len = 1;
    else
        line_len = h * theta;
    end
    line_len = max(1, h * theta);
    plot_3d_arrow(q, direction, line_len, 'LineSpec', 'k', 'LineWidth', 1, args.ForwardArgs{:});

    % Plot the helix
    % This will make a vertical helix... but what if I want something on
    % its side? And this vertical one goes the other direction anyway
    % because our current example has the z upside down.
    % So this would need a lot of work.
    % I think you'd have to pass in a starting point? And get radius and
    % starting rads that way?
%     starting_rads = 0;  % TODO: get this from somewhere
%     radius = 1;   % TODO: get this from somewhere
%     rads = linspace(0, theta, 200);
%     x = radius * sin(rads + starting_rads) + q(1);
%     y = radius * cos(rads + starting_rads) + q(2);
%     z = rads * h + q(3);
%     plot3(x, y, z, 'k:');
    
    if ~isnan(args.Label)
        text(origin(1), origin(2), origin(3), args.Label, 'Interpreter', 'latex');
    end

    if ~was_hold
        hold off;
    end
