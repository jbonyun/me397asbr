function plot_manipulability_ellipsoid(Jpart, varargin)
    % Plots manipulability ellipsoid of the given 3D piece of a Jacobian
    %  This is used as a helper for linear and angular versions.
    % Inputs:
    %   Jpart: 3xdof partial jacobian matrix
    % Outputs: none
    % Jeff Bonyun (jb79332), jbonyun@utexas.edu, 20220401
    % On behalf of the Sun/Bonyun team for ME397 ASBR, Spring 2022.
    % Source: Alambeigi, F. ASBR Lecture Notes. 2022, W8-L1, p 5.
    % Source of RU&ssc for rotation: https://math.stackexchange.com/a/897677/283227

    p = inputParser;
    isAxes = @(x) isa(x, 'matlab.graphics.axis.Axes');
    addRequired(p, 'Jpart'); %, @(x)(validateattributes(x, {'numeric', 'real', 'nonempty', 'nonnan', 'finite', '2d', 'rows', 3})));
    addOptional(p, 'Axis', gca, isAxes);
    %addParameter(p, 'scale', 1., @isfloat);
    parse(p, Jpart, varargin{:});
    args = p.Results;

    assert(size(args.Jpart,1) == 3, 'Jpart must be in R3');

    A = args.Jpart * args.Jpart';
    [evec, eval] = eig(A);
    % Make eval a vector instead of diagonal matrix.
    eval = diag(eval);
    % Make sure it is semi-PD
    assert(all(min(eval) >= 0), 'J*JT matrix is not positive-definite');

    % Calculate the rotations for the ellipse.
    % Rotate what was +x to the direction of largest evec
    ssc = @(v) [0 -v(3) v(2); v(3) 0 -v(1); -v(2) v(1) 0];
    RU = @(A,B) eye(3) + ssc(cross(A,B)) + ssc(cross(A,B))^2*(1-dot(A,B))/(norm(cross(A,B))^2);
    if norm([1 0 0]' - evec(:,3)) < 1e-7
        % No rotation needed (which messes up the calculation otherwise)
        axis1 = evec(:,3);
        angle1 = 0;
        rot1 = eye(3);
    else
        rot1 = RU([1 0 0]', evec(:,3));
        aa1 = rot2aa(rot1);
        axis1 = aa1(1:3);
        angle1 = aa1(4);
    end
    % Where does the old y -- which was the second largest axis of the
    yafter1 = rot1 * [0; 1; 0];
    % Rotate that to point in the direction of evec2.
    % Rotation axis is perp to both its current dir and target dir.
    if norm(yafter1 - evec(:,2)) < 1e-7
        % No rotation needed (which messes up the calculation otherwise)
        axis2 = evec(:,2);
        angle2 = 0;
        rot2 = eye(3);
    else
        rot2 = RU(yafter1, evec(:,2));
        aa2 = rot2aa(rot2);
        axis2 = aa2(1:3);
        angle2 = aa2(4);
    end
    % Third dimension (the one that was originally z) is the smallest
    % dimension, and it should now point to the third eigen vector.
    zafter2 = rot2 * rot1 * [0; 0; 1];

    % Make the ellipse shape, using sqrt(evals) as radius in each dim.
    [x,y,z] = ellipsoid(0, 0, 0, sqrt(eval(3)), sqrt(eval(2)), sqrt(eval(1)), 100);
    % Rotate the ellipse so its principal axes align with eigenvectors.
    xyz = cat(3, x, y, z);
    xyz = permute(xyz, [3 2 1]);
    rotxyz = nan(size(x,1), size(x,2), 3);
    for i = 1:size(xyz,3)
        rotxyz(:, i, :) = (rot2 * rot1 * xyz(:,:,i))';
    end

    % Plot it!
    mesh(args.Axis, rotxyz(:,:,1), rotxyz(:,:,2), rotxyz(:,:,3));
    % Alternative to manually rotating to get rotxyz is to plot it and then
    % use matlab to rotate the graphics. I prefered explicit version.
    %s = mesh(x, y, z);
    %rotate(s, axis1, rad2deg(angle1));
    %rotate(s, axis2, rad2deg(angle2));
    
    hold all;
    arrow_ends = evec * diag(sqrt(eval));
    quiver3(0,0,0,arrow_ends(1,3),arrow_ends(2,3),arrow_ends(3,3), 'off', 'r');
    quiver3(0,0,0,arrow_ends(1,2),arrow_ends(2,2),arrow_ends(3,2), 'off', 'g');
    quiver3(0,0,0,arrow_ends(1,1),arrow_ends(2,1),arrow_ends(3,1), 'off', 'b');
    hold off;

    newlim = max(max(abs([xlim; ylim; zlim])));
    xlim([-newlim newlim]);
    ylim([-newlim newlim]);
    zlim([-newlim newlim]);
    %axis equal;
