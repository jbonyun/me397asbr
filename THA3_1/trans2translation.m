function [translation] = trans2translation(trans)
    % Extracts the translation from a 4x4 transformation.
    assert(all(trans(4,:) == [0 0 0 1]), 'Not given a valid homogenous transformation matrix because last row is wrong');
    translation = trans(1:3,4);
