function [screw, bscrew] = calc_screw(dof, offset, axes, home)

    % Calculate the screw vectors from each joint's axis at home and translation
    % at home.
    screw = nan(6, dof);
    for i = 1:dof
        v = -cross(axes(i,:)', offset(i,:)');
        screw(:, i) = [axes(i,:)'; v];
    end
    
    % Calculate the body screw matrix.
    % Uses adjoint of inverse home transform to change space screws to body.
    bscrew = nan(6, dof);
    for i = 1:dof
        bscrew(:, i) = adjoint_transform(inv(home)) * screw(:, i);
    end