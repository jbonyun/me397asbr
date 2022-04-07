function ad = adjoint_transform(trans)
    % Returns adjoint of given transformation matrix.
    % Inputs:
    %   trans: 4x4 homogenous transformation or configuration
    % Outputs:
    %   ad: 6x6 adjoint of transformation
    % Jeff Bonyun (jb79332), jbonyun@utexas.edu, 20220323
    % On behalf of the Sun/Bonyun team for ME397 ASBR, Spring 2022.

    assert(size(trans,1) == 4, 'Must have 4 rows to be transform');
    assert(size(trans,2) == 4, 'Must have 4 columns to be transform');
    assert(all(trans(4,:) == [0 0 0 1], "all"), 'Last col must be [0 0 0 1]');

    ad = [trans(1:3,1:3) zeros(3,3); skewsym(trans(1:3,4)) * trans(1:3,1:3) trans(1:3,1:3)];