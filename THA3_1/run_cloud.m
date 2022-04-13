clear;

trial_letter = 'b';

% Read calbody file
calbodyfname = sprintf('data/pa1-debug-%s-calbody.txt', trial_letter);
cb = read_calbody(calbodyfname);

% Read calreadings file
calreadfname = sprintf('data/pa1-debug-%s-calreadings.txt', trial_letter);
cr = read_calreadings(calreadfname);

% Read the example output file, which we should be able to match
outtruthfname = sprintf('data/pa1-debug-%s-output1.txt', trial_letter);
outtruth = read_output(outtruthfname);

%% Tac: transform from optical sensor frame (a) to calibration object frame (c)
% Uses optical observations of optical points on calibration object.

%figure;
for f = 1:numel(cr.frames)
    Tac{f} = correspondence_points(cb.a,  cr.frames{f}.a);
    %plot_3d_axis_transform(Tac{f}, 'scale', 30);
    % Find the projection of the points to a-space
    reproj_in_a = dehomogenize(Tac{f} * homogenize(cb.a'));
    diffs = reproj_in_a - cr.frames{f}.a';
    pt_reproj_error = sum(sqrt(diffs.^2));
    assert(all(pt_reproj_error < 0.1, 'all'), 'Reprojection not very accurate');
    fprintf('Norm reprojection error of Tac: %f\n', norm(pt_reproj_error));
end

%% Tad: transform from optical sensor frame (a) to EM base frame (d)
% Uses optical observations of optical points on the EM base station.

%figure;
for f = 1:numel(cr.frames)
    Tad{f} = correspondence_points(cb.d,  cr.frames{f}.d);
    %plot_3d_axis_transform(Tcd{f}, 'scale', 30);
    % Find the projection of the points to a-space
    reproj_in_a = dehomogenize(Tad{f} * homogenize(cb.d'));
    diffs = reproj_in_a - cr.frames{f}.d';
    pt_reproj_error = sum(sqrt(diffs.^2));
    assert(all(pt_reproj_error < 0.1, 'all'), 'Reprojection not very accurate');
    fprintf('Norm reprojection error of Tad: %f\n', norm(pt_reproj_error));
end

%% Tdc: transform from EM base frame (d) to calibration object frame (c)
% Uses EM observations of EM tracking points on calibration object.
% We don't actually use this. Instead we figure out what the optical
% tracker would expect the EM sensor to read.

%figure;
for f = 1:numel(cr.frames)
    Tdc{f} = correspondence_points(cb.c,  cr.frames{f}.c);
    %plot_3d_axis_transform(Tdc{f}, 'scale', 30);
    % Find the projection of the points to d-space
    reproj_in_d = dehomogenize(Tdc{f} * homogenize(cb.c'));
    diffs = reproj_in_d - cr.frames{f}.c';
    pt_reproj_error = sum(sqrt(diffs.^2));
    %assert(all(pt_reproj_error < 0.1, 'all'), 'Reprojection not very accurate');
    fprintf('Norm reprojection error of Tdc: %f\n', norm(pt_reproj_error));
end


%% Calculate implied Ci, via Tad and Tac
% This is the location EM tracking points on calibration object wrt to EM
% base station. But it uses the optical tracker's measurements of where the
% calibration object is and where the EM base station is.

% Prepare output
out.NC = cr.NC;
out.Nframes = cr.Nframes;
% Implied by Tac and Tad
for f = 1:numel(cr.frames)
    Tdc_calc{f} = inv(Tad{f}) * Tac{f};
    out.frames{f}.C = dehomogenize(Tdc_calc{f} * homogenize(cb.c'))';
end

%% Write the output file
% This can be compared to the debug output1 files provided in the
% assignment.

outfname = sprintf('data/pa1-debug-%s-outtest.txt', trial_letter);
write_output(outfname, out);

%% Compare our output to provided output

for f = 1:numel(cr.frames)
    %[out.frames{f}.C  outtruth.frames{f}.C out.frames{f}.C - outtruth.frames{f}.C sqrt(sum((out.frames{f}.C - outtruth.frames{f}.C).^2, 2))]
    fprintf('RMS distance from truth to our calc for points in frame: %f\n', sqrt(mean(sum((out.frames{f}.C - outtruth.frames{f}.C).^2, 2),1)));
    fprintf('Worst point-point distance in frame: %f\n', max(sqrt(sum((out.frames{f}.C - outtruth.frames{f}.C).^2, 2))));
    %fprintf('Norm of all dimension / all point errors: %f\n', norm(out.frames{f}.C - outtruth.frames{f}.C));
end