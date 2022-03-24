function i = iseye(m)
    % Returns logical 1/0 for whether m is the identity matrix

    if ~ismatrix(m)
        i = false;
    elseif size(m,1) ~= size(m,2)
        i = false;
    else
        i = all(abs(m - eye(size(m,1))) < 1e-7, 'all');
    end