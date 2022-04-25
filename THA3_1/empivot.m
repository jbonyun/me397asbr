function [btip, bpost, rmsPostError, worstDRFErr] = empivot(pivot_data)

    % We get our base truth of marker locations from first frame.
    % All the other frames are rotations of this.
    G0 = mean(pivot_data.frames{1}.G);
    gj = pivot_data.frames{1}.G - repmat(G0, pivot_data.NG, 1);
    
    % Run the calibration
    [btip, bpost, FG, drfRMSErr, postErr] = pivot_calibration(gj, cellfun(@(x) x.G, pivot_data.frames, 'UniformOutput', false));

    do_plot = true;
    if do_plot
        % Plot a picture of the calibration poses
        plot_pivot_calibration(FG, btip, bpost);
        title(sprintf('EM Pivot Calibration Poses, Scenario %s', pivot_data.letter));
    end
    
    % What was the typical error between tip and post?
    rmsPostError = sqrt(mean(postErr.^2));
    % What was the worst rms error for the markers on the pointer, from all the
    % frames?
    worstDRFErr = max(drfRMSErr);
    % How long is the pointer? For sanity check.
    pointer_len = norm(btip);
    %fprintf('EM Pivot   Scenario %s   RMS Post Err: %.3f   Max DRF RMS Err: %.3f   Pointer Len: %.3f\n', pivot_data.letter, rmsPostError, worstDRFErr, pointer_len);
