function b = isrot(rot)
    % Returns logical 1/0 for whether rot is a valid rotation matrix.

    dim = ismatrix(rot);
    szr = size(rot,1) == 3;
    szc = size(rot,2) == 3;
    det1 = logical(abs(det(rot) - 1) < 1e-7);
    i = iseye(rot * rot');
    b = dim && szr && szc && det1 && i;