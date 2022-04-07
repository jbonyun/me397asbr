function i = iseye(m, eps)
    % Returns logical 1/0 for whether m is the identity matrix

    if nargin < 2 || isnan(eps)
        eps = 1e-7;
    end

    if ~ismatrix(m)
        i = false;
    elseif size(m,1) ~= size(m,2)
        i = false;
    else
        i = all(abs(m - eye(size(m,1))) < eps, 'all');
    end