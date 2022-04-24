function [btip, bpost, Ts, rmsDRFErr, postErr] = pivot_calibration(truth_points, frames)
    % Finds the transformation between two corresponding sets of points.
    % Inputs:
    %    truth_points: nx3 matrix of x/y/z location of n points that define
    %                  the ground truth locations of markers on the pointer.
    %    frames: 1xk cell array of nx3 matrices of x/y/z location of n
    %            markers on the pointer in frame k.
    %            Hint: you can use this to get a field out of structs in a
    %                  cell array
    %              cellfun(@(x) x.my_field_name, my_cell_array, 'UniformOutput', false)
    % Outputs:
    %    btip:  3x1 translation from truth_point center to pointer tip.
    %    bpost: 3x1 translation from sensor origin to pivot post/dimple.
    %    Ts:    1xk cell array of 4x4 transformations from(?) sensor frame
    %           to(?) pointer center.
    %    rmseDRFErr: kx1 vector or root mean square error of each pose
    %                compared to the truth_points description of pointer
    %                markers.
    %    postErr: kx1 vector of distance (in mm) from the tip in frame k to
    %             the post. A measure of the quality of fit.
    % Jeff Bonyun (jb79332), jbonyun@utexas.edu, 20220424
    % On behalf of the Sun/Bonyun team for ME397 ASBR, Spring 2022.
    % Source: Alambeigi, F. ASBR Lecture Notes. 2022, W12-L1 p. 3.
    
    Nframes = numel(frames);
    Ts = cell(1, Nframes);
    rmsDRFErr = nan(Nframes, 1);
    bigleft = nan(Nframes * 3, 6);
    bigright = nan(Nframes * 3, 1);
    for k = 1:Nframes
        frame_points = frames{k};
        % Calculate transformation from base truth to observations
        Ts{k} = correspondence_points(truth_points, frame_points);
        % How error-full was that?
        reprojGframe = dehomogenize(Ts{k} * homogenize(truth_points'))';
        reprojErr = vecnorm((reprojGframe - frame_points)');
        rmsDRFErr(k,1) = sqrt(mean(reprojErr.^2));
        % Fill up the regression matrices
        bigleft(3*k-2:3*k, 1:3) = trans2rot(Ts{k});
        bigleft(3*k-2:3*k, 4:6) = -eye(3);
        bigright(3*k-2:3*k, 1) = -trans2translation(Ts{k});
    end
    
    % Run the regression to get btip and bpost
    x = bigleft \ bigright;
    btip = x(1:3);  % Location of tip wrt DRF on pointer (defined by first frame).
    bpost = x(4:6);  % Location of post wrt tracker.

    % The distance between each tip and the post we decided on.
    postErr = vecnorm(reshape(bigleft * x - bigright, [3 12]), 2, 1)';