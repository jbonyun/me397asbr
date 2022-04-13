function [T] = correspondence_points(ptsA, ptsB)
    % Finds the transformation between two corresponding sets of points.
    % Inputs:
    %    ptsA: nx3 matrix of x/y/z location of n points in frame A
    %    ptsB: nx3 matrix of x/y/z location of n points in frame B
    % Outputs:
    %    T:    4x4 transformation from A to B frames.
    % Jeff Bonyun (jb79332), jbonyun@utexas.edu, 20220409
    % On behalf of the Sun/Bonyun team for ME397 ASBR, Spring 2022.
    % Source: Alambeigi, F. ASBR Lecture Notes. 2022, W10-L2 pp. 12-18. 

    assert(all(size(ptsA) == size(ptsB)), 'Must be same sizes');
    assert(size(ptsA, 2) == 3, 'Must have 3 columns');

    n = size(ptsA, 1);

    abar = mean(ptsA, 1);
    bbar = mean(ptsB, 1);

    atilde = ptsA - repmat(abar, n, 1);
    btilde = ptsB - repmat(bbar, n, 1);

    H = zeros(3,3);
    for i = 1:n
        Hi = atilde(i, :)' * btilde(i, :);

        H = H + Hi;
    end

    % Eig methd
    delta = [H(2,3)-H(3,2); H(3,1)-H(1,3); H(1,2)-H(2,1)];
    G = [trace(H) delta'; delta H+H' - trace(H) * eye(3)];
    [evec, eval] = eig(G);
    biggest = find(max(diag(eval)) == diag(eval), 1, 'first');
    quatR = evec(:, biggest);
    R = q2rot(quatR);

    % SVD method
%     [U, ~, V] = svd(H);
%     R = V * U';
%     assert(det(R) - 1 < 1e-12, 'Det of rotation matrix is not 1, which indicates failed SVD');

    p = bbar' - R*abar';
    T = rottranslation2trans(R, p);
