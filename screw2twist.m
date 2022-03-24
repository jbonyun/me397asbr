function twist = screw2twist(screw, theta)
    % Converts a screw axis vector and a rotation angle around it
    % into the geometric interpretation of a screw.
    % Inputs:
    %   screw: screw vector, 6x1
    %   theta: amount to move around screw, scalar    
    % Outputs:
    %   twist: twist vector, 6x1 (stacks angular velocity over linear)
    % Jeff Bonyun (jb79332), jbonyun@utexas.edu, 20220215
    % On behalf of the Sun/Bonyun team for ME397 ASBR, Spring 2022.
    % Source: note taken during ASBR lecture W5-L1.

    assert(size(screw,1) == 6);
    assert(numel(theta) == size(screw,2));

    % Yep, it's simple.
    twist = screw * theta;