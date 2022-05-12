function [dq] = constrained_IK_step(robot, start_angles, pgoal, constraint_center, lr, varargin) %twist_to_dest, lr)
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

    p = inputParser;
    addParameter(p, 'use_joint_limits', true, @islogical);
    addParameter(p, 'joint_vel_limit', nan, @isfloat);
    addParameter(p, 'max_distance_from_goal', nan, @isfloat);
    addParameter(p, 'plane', []);
    addParameter(p, 'enforce_plane', nan);
    addParameter(p, 'weight_loc', 1.0, @isfloat);
    addParameter(p, 'weight_kinetic', 0, @isfloat);
    addParameter(p, 'weight_jointcenter', 0.0, @isfloat);
    addParameter(p, 'weight_orientation', 0.0, @isfloat);
    %p.KeepUnmatched = true;
    parse(p, varargin{:});
    args = p.Results;
    if isnan(args.enforce_plane)
        args.enforce_plane = ~isempty(args.plane);
    end
    if isempty(args.plane)
        args.plane_normal = [];
        args.plane_point = [];
    else
        args.plane_normal = args.plane(:,1)';
        args.plane_point = args.plane(:,2);
    end

    if isnan(constraint_center)
        constraint_center = pgoal;
    end

    % Apply FK to find tool tip pose in space frame.
    startTs = FK_space(robot, start_angles);
    % Get rotation matrix of current pose in space frame
    R = trans2rot(startTs);
    % Get t vector (translation of tip wrt space frame).
    tipZ = [0; 0; 100];  % The tool tip from end effector in body frame.
    tipTb = rottranslation2trans(eye(3), tipZ);
    Pkins = trans2translation(startTs);  % end effector translation in space frame before moving
    t = trans2translation(startTs * tipTb);

    % Split the Jacobian into translation and rotation parts.
    Js = J_space(robot, start_angles);
    Jalpha = Js(1:3, :);
    Jeps = Js(4:6, :);
    Jb = J_body(robot, start_angles);
    Jbalpha = Jb(1:3, :);
    Jbeps = Jb(4:6, :);

    % Objective function
    % Objective to minimize square change in each joint.
    Ckinetic = eye(7);
    dkinetic = zeros(7, 1);
    
    % Objective to minimize the translation distance to pgoal.
    Cloc = -skewsym(t) * Jalpha + Jeps;
    dloc = pgoal - t;  % The change in tip location we are seeking.

    Cjointcenter = eye(7);
    djointcenter = (robot.joint_limits(:,2)-robot.joint_limits(:,1))/2 - start_angles;

    % Objective to minimize the change in tool rotation from step to step.
    
    Corient = -skewsym(R*tipZ) * Jalpha;
    dorient = [0;0;0];


    % Limits
    % Limit joint range.
    % This will require qLA*x <= qLb and qUA*x <= qUb
    % Which in turn means x >= qLb and x <= qUb
    qL = robot.joint_limits(:, 1);
    qU = robot.joint_limits(:, 2);
    qLA = -eye(7);
    qLb = start_angles - qL;
    qUA = eye(7);
    qUb = qU - start_angles;

    % Limit joint motion in any one step
    dqUA = eye(7);
    dqUb = repmat(args.joint_vel_limit, 7, 1);
    dqLA = -eye(7);
    dqLb = repmat(args.joint_vel_limit, 7, 1);

    % Limit allowed distance from target.
    % Stay within `max_distance` of target point at all times.
    % Linearize by turning the sphere surrounding the target point into a
    % set of polygons.
    m = 10; n = 10;  % Count of polygon mesh; bigger is a better approximation.
    [polyalpha,polybeta] = meshgrid(linspace(0, 2*pi * (1 - 1/m), m), linspace(0, 2*pi * (1 - 1/n), n));
    polyalpha = reshape(polyalpha, [], 1);
    polybeta = reshape(polybeta, [], 1);
    poly = [cos(polyalpha).*cos(polybeta) cos(polyalpha).*sin(polybeta) sin(polyalpha)];
    polyA = poly * (R*((-skewsym(tipZ)*Jbalpha)+Jbeps));
    polyb = repmat(args.max_distance_from_goal, m*n, 1) - poly * (R*tipZ + Pkins - pgoal);

    plane_distance = 0;
    if isempty(args.plane_normal)
        planeA = [];
        planeb = [];
    else
        planeA = -args.plane_normal * R *(-skewsym(tipZ) * Jbalpha + Jbeps);
        planeb = args.plane_normal * t - args.plane_normal * args.plane_point - plane_distance;
    end


    % Set up the linear least squares problem.
    %   C,d s.t. we minimize norm(Cx - d)
    %   A,b s.t. we require Ax <= b
    %  Solves x = lsqlin(C,d,A,b);

    C = []; d = [];
    if args.weight_kinetic ~= 0; C = [C; args.weight_kinetic .* Ckinetic]; d = [d; args.weight_kinetic .* dkinetic]; end
    if args.weight_loc ~= 0; C = [C; args.weight_loc .* Cloc]; d = [d; args.weight_loc .* dloc]; end
    if args.weight_jointcenter ~= 0; C = [C; args.weight_jointcenter .* Cjointcenter]; d = [d; args.weight_jointcenter .* djointcenter]; end
    if args.weight_orientation ~= 0; C = [C; args.weight_orientation .* Corient]; d = [d; args.weight_orientation .* dorient]; end

    A = zeros(1, robot.dof); b = 0;  % If no other limits, need something or it crashes;
    if args.use_joint_limits A = [A; qLA; qUA]; b = [b; qLb; qUb]; end
    if ~isnan(args.joint_vel_limit) A = [A; dqLA; dqUA]; b = [b; dqLb; dqUb]; end
    if ~isnan(args.max_distance_from_goal) A = [A; polyA]; b = [b; polyb]; end
    if args.enforce_plane A = [A; planeA]; b = [b; planeb]; end
    
    % Solve the optimization problem.
    optopt = optimoptions(@lsqlin, 'Algorithm', 'interior-point', 'Display', 'off');
    [dq, resnorm, residual, exitflag, output, lambda] = lsqlin(C, d, A, b, [], [], [], [], [], optopt);
    % Did that work?
    if exitflag == -2
        % It failed to work because it is infeasible.
        % Try again, but without the polyhedron/sphere constraint.
        prox_active_word = 'inactive';
        A = zeros(1, robot.dof); b = 0;  % If no other limits, need something or it crashes;
        if args.use_joint_limits A = [A; qLA; qUA]; b = [b; qLb; qUb]; end
        if ~isnan(args.joint_vel_limit) A = [A; dqLA; dqUA]; b = [b; dqLb; dqUb]; end
        if args.enforce_plane A = [A; planeA]; b = [b; planeb]; end
        % Solve it again.
        [dq, resnorm, residual, exitflag, output, lambda] = lsqlin(C, d, A, b, [], [], [], [], [], optopt);
    else
        prox_active_word = 'active';
        % Did we get inside the sphere?
        %dist = norm(trans2translation(FK_space(robot, start_angles+dq)) - pgoal);
        %if dist > max_distance
        %    fprintf('After optimization with sphere constraint, solution was %.3fmm from goal, which is outside the sphere\n', dist);
        %end
    end
    
    if numel(dq) == 0
        assert(numel(dq) > 0);
    end

    % Translation before and after
    pafter = trans2translation(FK_space(robot, start_angles+dq));
    ptipafter = trans2translation(FK_space(robot, start_angles+dq) * tipTb);
    %[t pafter pgoal pafter - pgoal]

    % Final distance if all the linearization we assumed holds (assumes lr=1).
    linearized_dist = norm(Cloc*dq-dloc);
    % Final distance for real (assumes lr=1)
    dist = norm(pafter - pgoal);

    % Did we keep bounds?
    if ~all(A*dq<=(b+1e-8))
        missed_constraint = A*dq > (b+1e-8);
        fprintf('Didnt meet %d of the constraints\n', sum(A*dq>(b+1e-8)));
        fprintf('Missed constraint numbers: %s\n', mat2str([find(missed_constraint)]));
        %[A*dq b A*dq<=(b+1e-8)]
    end

    % Check if we're up against a joint limit, and display that.
    joint_limits_hit = sum(qUA*dq - qUb > -1e-4) + sum(qLA*dq - qLb > -1e-4);
    if joint_limits_hit > 0
        joint_limit_word = 'Hitting joint limit';
        fprintf('Joint limits exceeded on joints: %s\n', mat2str([find(qLA*dq - qLb > -1e-4)'  find(qUA*dq - qUb > -1e-4)']'));
    else
        joint_limit_word = '';
    end

    % Report
    fprintf('     Prox %10s  Dist: %.2f  TheorDist: %.2f  # Unmet const: %d   Joint limits hit: %d\n', prox_active_word, dist, linearized_dist, sum(A*dq>(b+1e-8)), joint_limits_hit);

    % Reduce the returned dq by the learning rate.
    dq = dq * lr;
    