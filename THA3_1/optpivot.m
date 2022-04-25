function [btip, bpost, rmsPostError, worstDRFErr] = optpivot(pivot_data, ground_truth_d)

    % Preallocate.
    FD = cell(1, pivot_data.Nframes);
    HinFD = cell(1, pivot_data.Nframes);
    
    % Calculate and apply the transformation to the EM base frame of
    % reference, using the optical markers on the EM base station.
    for f = 1:pivot_data.Nframes
        FD{f} = correspondence_points(ground_truth_d,  pivot_data.frames{f}.D);
        %plot_3d_axis_transform(Tad{f}, 'scale', 30);
        % Find the projection of the points to a-space
        reproj_in_a = dehomogenize(FD{f} * homogenize(ground_truth_d'));
        diffs = reproj_in_a - pivot_data.frames{f}.D';
        pt_reproj_error = sum(sqrt(diffs.^2));
        assert(all(pt_reproj_error < 0.1, 'all'), 'Reprojection not very accurate');
        %fprintf('Norm reprojection error of Tad: %f\n', norm(pt_reproj_error));
    
        % Transform all the optical observations of the pointer into frame FD
        HinFD{f} = dehomogenize(inv(FD{f}) * homogenize(pivot_data.frames{f}.H'))';
    end

    H0 = mean(HinFD{1});
    hj = HinFD{1} - repmat(H0, pivot_data.NH, 1);

    [btip, bpost, FHinFD, drfRMSErr, postErr] = pivot_calibration(hj, HinFD);

    do_plot = true;
    if do_plot
        % Plot a picture of the calibration poses
        plot_pivot_calibration(FHinFD, btip, bpost);
        title(sprintf('Optical Pivot Calibration Poses, Scenario %s', pivot_data.letter));
    end

    % What was the typical error between tip and post?
    rmsPostError = sqrt(mean(postErr.^2));
    % What was the worst rms error for the markers on the pointer, from all the
    % frames?
    worstDRFErr = max(drfRMSErr);
    % How long is the pointer? For sanity check.
    pointer_len = norm(btip);
    fprintf('Opt Pivot  Scenario %s   RMS Post Err: %.3f   Max DRF RMS Err: %.3f   Pointer Len: %.3f\n', pivot_data.letter, rmsPostError, worstDRFErr, pointer_len);
