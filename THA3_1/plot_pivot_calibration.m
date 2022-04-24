function plot_pivot_calibration(Ts, btip, bpost)
    % Plot each frame's pointer pose as an axis at its center.
    % Plot each frame's tip location as a dot.
    % Plot the post location as a dot.
    % All the dots ought to be very near each other, with the frames clustered
    % around it.

    figure;
    view(-10, 60);
    hold all;
    for k = 1:numel(Ts)
        % Draw origin axis for the pointer in this frame.
        plot_3d_axis_transform(Ts{k}, 'scale', 30);
        thistip = dehomogenize(Ts{k} * homogenize(btip));
        thisDRF = trans2translation(Ts{k});
        % Draw lines for the pointer from center to tip.
        plot3([thisDRF(1), thistip(1)], [thisDRF(2) thistip(2)], [thisDRF(3) thistip(3)], 'k.-', 'MarkerSize', 10);
    end
    % Plot the post.
    plot3(bpost(1), bpost(2), bpost(3), 'r*', 'MarkerSize', 15, 'LineWidth', 2);
    xlabel('x (mm)'); ylabel('y (mm)'); zlabel('z (mm)');
    title('Pivot Calibration Poses');
    hold off;