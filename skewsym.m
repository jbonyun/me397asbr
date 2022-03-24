function ss = skewsym(vec)
    % Converts a vector into its skew symmetric matrix form.
    % Inputs:
    %   vec: 3x1 vector
    % Outputs:
    %   ss: 3x3 skew symmetric matrix
    % Jeff Bonyun (jb79332), jbonyun@utexas.edu, 20220209
    % On behalf of the Sun/Bonyun team for ME397 ASBR, Spring 2022.

    if all(size(vec) == [3 1]) || all(size(vec) == [1 3])
        ss = zeros(3,3);
        % Fill in the key values in upper triangular.
        ss(1, 2) = -vec(3);
        ss(1, 3) =  vec(2);
        ss(2, 3) = -vec(1);
        % Copy/negate from upper triangular to lower triangular.
        ss = triu(ss) + (-triu(ss))';
    elseif all(size(vec) == [6 1])
        ss = zeros(4,4);
        ss(1:3, 1:3) = skewsym(vec(1:3));
        ss(1:3, 4) = vec(4:6);
    else
        error('skewsym not implemented for dimensions of input vec. Must be 3x1 or 6x1');
    end

