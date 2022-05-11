function [dq] = constrained_step(robot, start_angles, pgoal, constraint_center, lr) %twist_to_dest, lr)
    % Calculates a path from startT to destT.
    % Respects the joint limits in the robot struct.
    % Input:
    %  robot:       struct of robot information
    %               dof: scalar number of joints
    %               joint_limits: dofx2 matrix with min and max
    %                             angles for each joint (rad).
    % start_angles: dofx1 vector of starting joint angles.
    % twist_to_dest: 6x1 twist from current location to target location.
    % Output:
    %  dq:          dofx1 vector of changes in joint angles

    if isnan(constraint_center)
        constraint_center = pgoal;
    end

    % Apply FK to find tool tip pose in space frame.
    startTs = FK_space(robot, start_angles);
    % Get rotation matrix of current pose in space frame
    R = trans2rot(startTs);
    % Get t vector (translation of tip wrt space frame).
    t = trans2translation(startTs);
    % Get vector pgoal from the destination location.
    %pgoal = trans2translation(twist2trans(twist_to_dest)) + t;
    currError = t - pgoal;

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
    polyA = [cos(polyalpha).*cos(polybeta) cos(polyalpha).*sin(polybeta) sin(polyalpha)];
    polyb = repmat(max_distance, m*n, 1) - polyA * (t - constraint_center);
    polyA = polyA * Jeps;  % Change from cartesian to joint space.
    

    % Limit joint motion in any one step
    joint_vel_limit = 0.1;
    dqUA = eye(7);
    dqUb = repmat(joint_vel_limit, 7, 1);
    dqLA = -eye(7);
    dqLb = repmat(joint_vel_limit, 7, 1);

    % Set up the linear least squares problem.
    %   C,d s.t. we minimize norm(Cx - d)
    %   A,b s.t. we require Ax <= b
    %  Solves x = lsqlin(C,d,A,b);

    % Objective to minimize square change in each joint.
    Ckinetic = eye(7);
    dkinetic = zeros(7, 1);
    weightkinetic = 0.1;
    
    % Objective to minimize the translation distance to pgoal.
    Cloc = -skewsym(t) * Jalpha + Jeps;
    dloc = pgoal - t;  % The change in tip location we are seeking.
    weightloc = 1;

    % Objective to minimize the change in tool rotation from step to step.
    Z = [0; 0; 100];  % The tool tip from end effector in body frame.
    Corient = -skewsym(R*Z) * Jalpha;
    dorient = [0;0;0];
    weightorient = 0;
    
    C = []; d = [];
    if weightkinetic ~= 0; C = [C; weightkinetic .* Ckinetic]; d = [d; weightkinetic .* dkinetic]; end
    if weightloc ~= 0; C = [C; weightloc .* Cloc]; d = [d; weightloc .* dloc]; end
    if weightorient ~= 0; C = [C; weightorient .* Corient]; d = [d; weightorient .* dorient]; end

    A = zeros(1, robot.dof); b = 0;  % If no other limits, need something or it crashes;
    A = [A; qLA; qUA]; b = [b; qLb; qUb];  % Include joint limits.
    A = [A; dqLA; dqUA]; b = [b; dqLb; dqUb];  % Include joint velocity limits.
    A = [A; polyA]; b = [b; polyb];  % Include polyhedron mesh limit.
    
    % Solve the optimization problem.
    optopt = optimoptions(@lsqlin, 'Algorithm', 'interior-point', 'Display', 'off');
    [dq, resnorm, residual, exitflag, output, lambda] = lsqlin(C, d, A, b, [], [], [], [], [], optopt);
    % Did that work?
    if exitflag == -2
        % It failed to work because it is infeasible.
        % Try again, but without the polyhedron/sphere constraint.
        fprintf('Removed proximity constraint in order to make progress\n');
        A = zeros(1, robot.dof); b = 0;  % If no other limits, need something or it crashes;
        A = [A; qLA; qUA]; b = [b; qLb; qUb];  % Include joint limits.
        A = [A; dqLA; dqUA]; b = [b; dqLb; dqUb];  % Include joint velocity limits.
        % Solve it again.
        [dq, resnorm, residual, exitflag, output, lambda] = lsqlin(C, d, A, b, [], [], [], [], [], optopt);
    else
        % Did we get inside the sphere?
        dist = norm(trans2translation(FK_space(robot, start_angles+dq)) - pgoal);
        if dist > max_distance
            fprintf('After optimization with sphere constraint, solution was %.3fmm from goal, which is outside the sphere\n', dist);
        end
    end
    
    if numel(dq) == 0
        assert(numel(dq) > 0);
    end

    % Did that work?
    % How close to desired solution?
    %[C*dq d C*dq-d]
    % Did we keep bounds?
    if ~all(A*dq<=(b+1e-8))
        fprintf('Didnt meet %d of the constraints\n', sum(A*dq>(b+1e-8)));
        [A*dq b A*dq<=(b+1e-8)]
    end
    
    % Translation before and after
    %[trans2translation(FK_space(robot, start_angles)) trans2translation(FK_space(robot, start_angles+dq)) pgoal trans2translation(FK_space(robot, start_angles+dq)) - pgoal]
    %norm(trans2translation(FK_space(robot, start_angles+dq)) - pgoal)

    % Reduce the returned dq by the learning rate.
    dq = dq * lr;
    