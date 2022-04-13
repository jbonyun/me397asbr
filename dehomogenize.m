function [y] = dehomogenize(x)
    assert(size(x, 1) == 4, 'Needs to have 4 rows to be dehomogenized');
    assert(all(x(4,:) - 1 < 1e-7, 'all'), 'Last row needs to be all 1s to be dehomogenized');
    y = x(1:3, :);
