function [trans] = rottranslation2trans(R, p)
    % Assembles a transformation from rotation and translation.
    % Inputs:
    %   R: 3x3 rotation matrix
    %   p: 3x1 translation vector
    % Outputs:
    %   trans: 4x4 translation matrix

    trans = [R p; 0 0 0 1];