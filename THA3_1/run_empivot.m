%% Pivot calibration on EM pointer.

clear;

trial_letter = 'a';  % 'a' through 'g' exist

% Read pivot data file.
em_fname = sprintf('data/pa1-debug-%s-empivot.txt', trial_letter);
data = read_empivot(em_fname);

% We get our base truth of marker locations from first frame.
% All the other frames are rotations of this.
G0 = mean(data.frames{1}.G);
gj = data.frames{1}.G - repmat(G0, data.NG, 1);

% Run the calibration
[btip, bpost, FG, drfRMSErr, postErr] = pivot_calibration(gj, cellfun(@(x) x.G, data.frames, 'UniformOutput', false));

% Plot a picture of the calibration poses
plot_pivot_calibration(FG, btip, bpost);
title(sprintf('EM Pivot Calibration Poses, Scenario %s', trial_letter));

% What was the typical error between tip and post?
rmsPostError = sqrt(mean(postErr.^2));
% What was the worst rms error for the markers on the pointer, from all the
% frames?
worstDRFErr = max(drfRMSErr);
% How long is the pointer? For sanity check.
pointer_len = norm(btip);
fprintf('Scenario %s   RMS Post Err: %.3f   Max DRF RMS Err: %.3f   Pointer Len: %.3f\n', trial_letter, rmsPostError, worstDRFErr, pointer_len);
