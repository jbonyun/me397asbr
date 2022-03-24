function vec = unskewsym(ss)
    % Converts a skew-symmetric matrix into vector form.
    % Inputs:
    %   ss: 3x3 or 4x4 skew symmetric matrix
    % Outputs:
    %   vec: 3x1 or 6x1 vector
    % Jeff Bonyun (jb79332), jbonyun@utexas.edu, 20220209
    % On behalf of the Sun/Bonyun team for ME397 ASBR, Spring 2022.
    % Source: Alambeigi, F. ASBR Lecture Notes. 2022, W2-L2 p. 2.

    if all(size(ss) == [3 3])
        % Assert that it is indeed skew-symmetric.
        assert(all(abs(triu(ss) - -triu(ss')) < 1e-9, 'all'), 'Not skew-symmetric because not negative-mirrored across diagonal');
        assert(all(diag(ss) == 0),'Not skew-symmetric because diagonal not 0');
    
        vec = zeros(3,1);
        vec(1, 1) = ss(3, 2);
        vec(2, 1) = ss(1, 3);
        vec(3, 1) = ss(2, 1);
    elseif all(size(ss) == [4 4])
        vec = zeros(6,1);
        vec(1:3) = unskewsym(ss(1:3, 1:3));
        vec(4:6) = ss(1:3, 4);
    else
        error('unskewsym not implemented for dimensions of input matrix. Must be 3x3 or 4x4');
    end

