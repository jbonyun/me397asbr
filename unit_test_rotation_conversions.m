% Unit tests for conversion functions
% Jeff Bonyun (jb79332), jbonyun@utexas.edu, 20220209
% On behalf of the Sun/Bonyun team for ME397 ASBR, Spring 2022.

% Cases created/verified with https://www.andre-gaschler.com/rotationconverter/

% Shortcut for asserting that all elements of a matrix are the same within
% floating point tolerance.
assert_near = @(x,y) assert(all(abs(x-y) < 1e-7, 'all'));

% Test skewsym
assert_near(skewsym([1; 2; 3]), [0 -3 2; 3 0 -1; -2 1 0]);
assert_near(skewsym([1 2 3]), [0 -3 2; 3 0 -1; -2 1 0]);  % Handles row vector even though we prefer column vector
% Test 

% Test rot2aa
assert_near(rot2aa([1 -8 4; 4 4 7; -8 1 4]/9), [-1/3; 2/3; 2/3; pi/2]);
assert_near(rot2aa([-7 -4 -4; -4 -1 8; -4 8 -1]/9), [1/3; -2/3; -2/3; pi]);  % There are two valid solutions: negative of this axis is also correct.
assert_near(rot2aa([0.356122094059555 0.479871654140435 -0.801810607110657; -0.801810607110657 0.597576308787222 0.001518387657450; 0.479871654140435 0.642359518282996 0.597576308787222]), [1/3; -2/3; -2/3; deg2rad(74)]);
assert_near(rot2aa([1 0 0; 0 1 0; 0 0 1]), [1; 0; 0; 0]);    % The angle is arbitrary. My test assumes you will return 1;0;0 for arbitrary rotation axis.
assert_near(rot2aa([1 0 0; 0 -1 0; 0 0 -1]), [1; 0; 0; pi]);
assert_near(rot2aa([-1 0 0; 0 1 0; 0 0 -1]), [0; 1; 0; pi]);
assert_near(rot2aa([-1 0 0; 0 -1 0; 0 0 1]), [0; 0; 1; pi]);
assert_near(rot2aa([1 0 0; 0 0 -1; 0 1 0]), [1; 0; 0; pi/2]);
assert_near(rot2aa([0 0 -1; 0 1 0; 1 0 0]), [0; -1; 0; pi/2]);
assert_near(rot2aa([0 1 0; -1 0 0; 0 0 1]), [0; 0; -1; pi/2]);

% Test rot2q
% Remember our quats are w;x;y;z order, with scalar (real) part first. Not
% everyone uses that convention.
assert_near(rot2q(eye(3)), [1; 0; 0; 0]); % identity
assert_near(rot2q([1 0 0; 0 -1 0; 0 0 -1]), [0; 1; 0; 0]);  % around x by pi
assert_near(rot2q([-1 0 0; 0 1 0; 0 0 -1]), [0; 0; 1; 0]);  % around y by pi
assert_near(rot2q([-1 0 0; 0 -1 0; 0 0 1]), [0; 0; 0; 1]);  % around z by pi
assert_near(rot2q([1 0 0; 0 0 -1; 0 1 0]), [sqrt(2)/2; sqrt(2)/2; 0; 0]);  % around x by pi/2
assert_near(rot2q([0 0 -1; 0 1 0; 1 0 0]), [sqrt(2)/2; 0; -sqrt(2)/2; 0]);  % around y by -pi/2
assert_near(rot2q([0 1 0; -1 0 0; 0 0 1]), [sqrt(2)/2; 0; 0; -sqrt(2)/2]);  % around z by -pi/2
assert_near(rot2q(aa2rot([sqrt(2)/2; sqrt(2)/2; 0; pi/2])), [sqrt(2)/2; 1/2; 1/2; 0]);  % around xy diagonal by pi/2
assert_near(rot2q([-0.2248451, 0.4912955, 0.8414710; 0.0599876, 0.8689247, -0.4912955; -0.9725462, -0.0599876, -0.2248451]), [0.5956582; 0.1810216; 0.7613499; -0.1810216]);

% Test rot2rpy
% This is the same as ZYX euler angles
assert_near(rot2rpy(eye(3)), [0; 0; 0]); % identity
assert_near(rot2rpy([1 0 0; 0 -1 0; 0 0 -1]), [pi; 0; 0]);  % around x by pi
assert_near(rot2rpy([-1 0 0; 0 1 0; 0 0 -1]), [pi; 0; pi]);  % around y by pi
assert_near(rot2rpy([-1 0 0; 0 -1 0; 0 0 1]), [0; 0; pi]);  % around z by pi
assert_near(rot2rpy([1 0 0; 0 0 -1; 0 1 0]), [pi/2; 0; 0]);  % around x by pi/2
try
    rot2rpy([0 0 -1; 0 1 0; 1 0 0]);  % around y by -pi/2
    had_err = false;
catch
    had_err = true;
end
assert(had_err, 'Must throw error at pi/2 pitch');
assert_near(rot2rpy([0 1 0; -1 0 0; 0 0 1]), [0; 0; -pi/2]);  % around z by -pi/2
assert_near(rot2rpy([1 -8 4; 4 4 7; -8 1 4]/9), [0.2449787; 1.0949141; 1.3258177]);
assert_near(rot2rpy([-1/3, 2/3, 2/3; 2/3, -1/3,  2/3; 2/3, 2/3, -1/3]), [2.0344439; -0.7297277; 2.0344439]);
assert_near(rot2rpy([1 0 0; 0 -1 0; 0 0 -1]), [pi; 0; 0]);  % around -x by pi
assert_near(rot2rpy([-1 0 0; 0 -1 0; 0 0 1]), [0; 0; pi]);  % around -z by pi

% Test rot2zyz
% ZYZ is not supported by that website tool, so need a new source of tests.
% I found Matlab's rotm2eul, which I will use just to verify correct answers.
assert_near(rot2zyz([1 0 0; 0 1 0; 0 0 1]), [0; 0; 0]);  % identity
try
    rot2zyz([1 0 0; 0 -1 0; 0 0 -1]);  % around x by pi
    had_err = false;
catch
    had_err = true;
end
assert(had_err, 'Must throw error for pi around x');
try
    rot2zyz([-1 0 0; 0 1 0; 0 0 -1]);  % around y by pi
    had_err = false;
catch
    had_err = true;
end
assert(had_err, 'Must throw error for pi around y');
try
    rot2zyz([-1 0 0; 0 -1 0; 0 0 1]);  % around z by pi
    had_err = false;
catch
    had_err = true;
end
assert(had_err, 'Must throw error for pi around z');
assert_near(rot2zyz([1 0 0; 0 0 -1; 0 1 0]), [-pi/2; pi/2; pi/2]);  % around x by pi/2
assert_near(rot2zyz([0 0 -1; 0 1 0; 1 0 0]), [pi; pi/2; pi]);  % around y by -pi/2
try
    rot2zyz([0 -1 0; 1 0 0; 0 0 1]);  % around z by pi/2
    had_err = false;
catch
    had_err = true;
end
assert(had_err, 'Must throw error for pi/2 around z');
%assert_near(rot2zyz([1 -8 4; 4 4 7; -8 1 4]/9), [-2.089942441041420;  -1.110242335113574;  -3.017237659043032]);
assert_near(rot2zyz([1 0 0; 0 0 1; 0 -1 0]), [pi/2; pi/2; -pi/2]);  % around -x by pi/2
try
    rot2zyz([0 1 0; -1 0 0; 0 0 1]);  % around -z by pi/2
    had_err = false;
catch
    had_err = true;
end
assert(had_err, 'Must throw error for pi/2 around z');

assert_near(rot2zyz([1 -8 4; 4 4 7; -8 1 4]/9), [1.051650212548374; 1.110242335113574; 0.124354994546761]);
assert_near(rot2zyz([-1/3, 2/3, 2/3; 2/3, -1/3,  2/3; 2/3, 2/3, -1/3]), [0.785398163397448; 1.910633236249019; 2.356194490192345]);
try
    rot2zyz(aa2rot([0; 0; 1; pi/2]));
    had_err = false;
catch
    had_err = true;
end
assert(had_err, 'rot2zyz must throw error at 0/pi second angle');


% Test that rot2zyz and zyz2rot are inverses.
assert_near(zyz2rot(rot2zyz([1 0 0; 0 1 0; 0 0 1])), [1 0 0; 0 1 0; 0 0 1]);
assert_near(zyz2rot(rot2zyz([1 -8 4; 4 4 7; -8 1 4]/9)), [1 -8 4; 4 4 7; -8 1 4]/9);
assert_near(zyz2rot(rot2zyz([-1/3, 2/3, 2/3; 2/3, -1/3,  2/3; 2/3, 2/3, -1/3])), [-1/3, 2/3, 2/3; 2/3, -1/3,  2/3; 2/3, 2/3, -1/3]);
assert_near(zyz2rot(rot2zyz(aa2rot([1; 0; 0; pi/8]))), aa2rot([1; 0; 0; pi/8]));
assert_near(zyz2rot(rot2zyz(aa2rot([0; 1; 0; pi/2]))), aa2rot([0; 1; 0; pi/2]));

% Test aa2rot
assert_near(aa2rot([-1/3, 2/3, 2/3, deg2rad(0)]'), [1 0 0; 0 1 0; 0 0 1]);  % 0 angle gives identity axis (not strictly required, but seems like a good default)
assert_near(aa2rot([324, -2343, 343, deg2rad(0)]'), [1 0 0; 0 1 0; 0 0 1]);  % 0 angle gives identity axis (not strictly required, but seems like a good default)
assert_near(aa2rot([-1/3, 2/3, 2/3, deg2rad(90)]'), [1 -8 4; 4 4 7; -8 1 4]/9);
assert_near(aa2rot([-1/3, 2/3, 2/3, deg2rad(180)]'), [-7 -4 -4; -4 -1 8; -4 8 -1]/9);
assert_near(aa2rot([-1/3, 2/3, 2/3, deg2rad(-74)]'), [0.3561221 0.4798717 -0.8018106; -0.8018106  0.5975763  0.0015184;   0.4798717 0.6423595  0.5975763]);
assert_near(aa2rot([1/3, -2/3, -2/3, deg2rad(74)]'), aa2rot([-1/3, 2/3, 2/3, deg2rad(-74)]'));  % Negate axis and negate angle = same thing.
assert_near(aa2rot([1; 0; 0; pi]), [1 0 0; 0 -1 0; 0 0 -1]);
assert_near(aa2rot([0; 1; 0; pi]), [-1 0 0; 0 1 0; 0 0 -1]);
assert_near(aa2rot([0; 0; 1; pi]), [-1 0 0; 0 -1 0; 0 0 1]);
assert_near(aa2rot([1; 0; 0; pi/2]), [1 0 0; 0 0 -1; 0 1 0]);
assert_near(aa2rot([0; -1; 0; pi/2]), [0 0 -1; 0 1 0; 1 0 0]);
assert_near(aa2rot([0; 0; -1; pi/2]), [0 1 0; -1 0 0; 0 0 1]);
assert_near(aa2rot([100; 0; 0; pi/2]), aa2rot([1; 0; 0; pi/2]));  % Axis doesn't have to be unit length; we just truncate it.
assert_near(aa2rot([-1/3, 2/3, 2/3, deg2rad(90)]), [1 -8 4; 4 4 7; -8 1 4]/9);  % Handles a row vector, even though we prefer column vectors

% Test q2rot
assert_near(q2rot([sqrt(2)/2; -0.2357023; sqrt(2)/3; sqrt(2)/3]), [1 -8 4; 4 4 7; -8 1 4]/9);
assert_near(q2rot([1; 0; 0; 0]), [1 0 0; 0 1 0; 0 0 1]);  % identity
assert_near(q2rot([0; 1; 0; 0]), [1 0 0; 0 -1 0; 0 0 -1]);  % around x by pi
assert_near(q2rot([0; 0; 1; 0]), [-1 0 0; 0 1 0; 0 0 -1]);  % around y by pi
assert_near(q2rot([0; 0; 0; 1]), [-1 0 0; 0 -1 0; 0 0 1]);  % around z by pi
assert_near(q2rot([sqrt(2)/2; sqrt(2)/2; 0; 0]), [1 0 0; 0 0 -1; 0 1 0]);  % around x by pi/2
assert_near(q2rot([sqrt(2)/2; 0; -sqrt(2)/2; 0]), [0 0 -1; 0 1 0; 1 0 0]);  % around y by -pi/2
assert_near(q2rot([sqrt(2)/2; 0; 0; -sqrt(2)/2]), [0 1 0; -1 0 0; 0 0 1]);  % around z by -pi/2
try
    q2rot([0; 0; 0; 0]);
    had_err = false;
catch
    had_err = true;
end
assert(had_err, 'q2rot must throw error when 0 length');
assert_near(q2rot([2; 0; 0; 0]), [1 0 0; 0 1 0; 0 0 1]);  % normalizes quat for identity
assert_near(q2rot([0; 2; 0; 0]), [1 0 0; 0 -1 0; 0 0 -1]);  % normalizes quat for around x
assert_near(q2rot([sqrt(2)/2 -0.2357023 sqrt(2)/3 sqrt(2)/3]), [1 -8 4; 4 4 7; -8 1 4]/9);  % Handles row vector even though we prefer column vectors
