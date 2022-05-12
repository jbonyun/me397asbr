function plot_3d_plane(plane_normal, plane_point, scale_fac)

    % Find equation of plane:
    %    Ax + By + Cz = D
    A = plane_normal(1);
    B = plane_normal(2);
    C = plane_normal(3);
    D = dot(plane_normal, plane_point * scale_fac);

    xl = xlim;
    yl = ylim;
    xs = [xl(1) xl(2) xl(2) xl(1)]' * .99;
    ys = [yl(1) yl(1) yl(2) yl(2)]' * .99;

    zs = (A.*xs + B.*ys - D) ./ -C;

    p = patch(xs, ys, zs, 'r');
    set(p, 'facealpha', 0.3);
