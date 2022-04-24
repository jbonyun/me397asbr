%% Pivot calibration test

%clear;

%trial_letter = 'a';

% Read calbody file (for ground truth positions of markers d on EM base)
calbodyfname = sprintf('data/pa1-debug-%s-calbody.txt', trial_letter);
cb = read_calbody(calbodyfname);

% Read pivot data file.
opt_fname = sprintf('data/pa1-debug-%s-optpivot.txt', trial_letter);
data = read_optpivot(opt_fname);

%% Tad: transform from optical sensor frame (a) to EM base frame (d)
% Uses optical observations of optical points on the EM base station.

%figure;
FD = cell(1, data.Nframes);
for f = 1:data.Nframes
    FD{f} = correspondence_points(cb.d,  data.frames{f}.D);
    %plot_3d_axis_transform(Tad{f}, 'scale', 30);
    % Find the projection of the points to a-space
    reproj_in_a = dehomogenize(FD{f} * homogenize(cb.d'));
    diffs = reproj_in_a - data.frames{f}.D';
    pt_reproj_error = sum(sqrt(diffs.^2));
    assert(all(pt_reproj_error < 0.1, 'all'), 'Reprojection not very accurate');
    %fprintf('Norm reprojection error of Tad: %f\n', norm(pt_reproj_error));
end

%% Transform all the optical observations of the pointer into frame FD

HinFD = cell(1, data.Nframes);
for f = 1:data.Nframes
    HinFD{f} = dehomogenize(inv(FD{f}) * homogenize(data.frames{f}.H'))';
end

%% Determine our fake ground truth for optical markers on pointer
% We get our ground truth of marker locations from first frame, because we
% do not know the ground truth about the pointer from the data files.

H0 = mean(HinFD{1});
hj = HinFD{1} - repmat(H0, data.NH, 1);

%% Run the calibration
[btip, bpost, FHinFD, drfRMSErr, postErr] = pivot_calibration(hj, HinFD);

% Plot a picture of the calibration poses
%plot_pivot_calibration(FHinFD, btip, bpost);
%title(sprintf('Optical Pivot Calibration Poses, Scenario %s', trial_letter));

% What was the typical error between tip and post?
rmsPostError = sqrt(mean(postErr.^2));
% What was the worst rms error for the markers on the pointer, from all the
% frames?
worstDRFErr = max(drfRMSErr);
% How long is the pointer? For sanity check.
pointer_len = norm(btip);
fprintf('Scenario %s   RMS Post Err: %.3f   Max DRF RMS Err: %.3f   Pointer Len: %.3f\n', trial_letter, rmsPostError, worstDRFErr, pointer_len);

