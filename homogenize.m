function [y] = homogenize(x)
    assert(size(x, 1) == 3, 'Needs to have 3 rows to be homogenized');
    y = [x; ones(1, size(x,2))];
