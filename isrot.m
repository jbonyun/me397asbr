function b = isrot(rot, eps)
    % Returns logical 1/0 for whether rot is a valid rotation matrix.
    if nargin < 2
        eps = 1e-7;
    end

    dim = ismatrix(rot);
    szr = size(rot,1) == 3;
    szc = size(rot,2) == 3;
    det1 = logical(abs(det(rot) - 1) < eps);
    i = iseye(rot * rot', eps);
    b = dim && szr && szc && det1 && i;