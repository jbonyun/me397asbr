function [dq] = move_with_joint_limits(robot, start_angles, destTs)
    % Calculates a path from startT to destT.
    % Respects the joint limits in the robot struct.
    % Input:
    %  robot:       struct of robot information
    %               dof: scalar number of joints
    %               joint_limits: dofx2 matrix with min and max
    %                             angles for each joint (rad).
    % start_angles: dofx1 vector of starting joint angles.
    % destTs:       4x4 target transformation matrix in space frame (which
    %               is the robot root).
    % Output:
    %  dq:          dofx1 vector of changes in joint angles

    % Apply FK to find tool tip pose in space frame.
    startTs = FK_space(robot, start_angles);
    % Get t vector (translation of tip wrt space frame).
    t = trans2translation(startTs);
    % Get vector pgoal from the destination location.
    pgoal = trans2translation(destTs);

    % Split the Jacobian into translation and rotation parts.
    Js = J_space(robot, start_angles);
    Jalpha = Js(1:3, :);
    Jeps = Js(4:6, :);

    % Joint limit bounds
    % This will require qLA*x <= qLb and qUA*x <= qUb
    % Which in turn means x >= qLb and x <= qUb
    qL = robot.joint_limits(:, 1);
    qU = robot.joint_limits(:, 2);
    qLA = -eye(7);
    qLb = start_angles - qL;
    qUA = eye(7);
    qUb = qU - start_angles;

    % Distance from target bounds.
    % Stay within `max_distance` of target point at all times.
    % Linearize by turning the sphere surrounding the target point into a
    % set of polygons.
    max_distance = 3;
    m = 10; n = 10;  % Count of polygon mesh; bigger is a better approximation.
    [polyalpha,polybeta] = meshgrid(linspace(0, 2*pi * (1 - 1/m), m), linspace(0, 2*pi * (1 - 1/n), n));
    polyalpha = reshape(polyalpha, [], 1);
    polybeta = reshape(polybeta, [], 1);
    polyA = [cos(polyalpha).*cos(polybeta) cos(polyalpha).*sin(polybeta) sin(polyalpha) zeros(m*n, 3)];
    polyA = polyA * Js;  % Change from cartesian to joint space.
    polyb = repmat(max_distance, m*n, 1);

    % Set up the linear least squares problem.
    %   C,d s.t. we minimize Cx - d
    %   A,b s.t. we require Ax <= b
    %  Solves x = lsqlin(C,d,A,b);
    C = -skewsym(t) * Jalpha + Jeps;
    d = pgoal - t;  % The change in tip location we are seeking.
    A = [];
    b = [];
    A = [A; polyA]; b = [b; polyb];  % Include polygon mesh limit.
    %A = [A; qLA; qUA]; b = [b; qLb; qUb];  % Include joint limits.

    % Solve the optimization problem.
    [dq, resnorm, residual, exitflag, output, lambda] = lsqlin(C, d, A, b);


    % Did that work?

    