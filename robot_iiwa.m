function [robot] = robot_iiwa()
    % Load the definition of the Kuka LBR iiwa R820 14.
     
    % Manually defined screw of all the things
    % Configuration of end effector in home position (straight up).
    robot.home = [eye(3,3) [0; 0; 1306]; 0 0 0 1];
    
    % In home position, the direction of the axis of rotation for each joint.
    % Positive rotations are CCW when looking in the given direction.
    robot.axes = [
        0  0  1;
        0  1  0;
        0  0  1;
        0 -1  0;
        0  0  1;
        0  1  0;
        0  0  1;
        ];
    % In home position, the location of the center of each joint, in mm, in
    % world coordinates (center of base on table).
    robot.offset = [
        0 0 170;
        0 0 360;
        0 0 600;
        0 0 780;
        0 0 1000;
        0 -60 1180;
        0 0 1271;
        ];
    
    % Extract DOF from the parameters we have already given.
    robot.dof = size(robot.axes,1);
    
    % Calculate the screw vectors from each joint's axis at home and translation
    % at home.
    for i = 1:robot.dof
        v = -cross(robot.axes(i,:)', robot.offset(i,:)');
        robot.screw(:, i) = [robot.axes(i,:)'; v];
    end
    
    % Calculate the body screw matrix.
    % Uses adjoint of inverse home transform to change space screws to body.
    for i = 1:robot.dof
        robot.bscrew(:, i) = adjoint_transform(inv(robot.home)) * robot.screw(:, i);
    end