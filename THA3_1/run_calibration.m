%#ok<*MINV> % Ignore inv warnings.

%% Manually select which scenario to run.
% Or you can set this variable before calling this module.
% If you are setting here, you might want to 'clear' as well, just for
% purity.

%clear;
%trial_letter = 'a';

%% Read all the data files.

% Decide if we are doing a "debug" file set or "unknown"
if double(trial_letter) <= double('g')
    file_set = 'debug';
    is_debug = true;
else
    file_set = 'unknown';
    is_debug = false;
end

% Read calbody file
calbodyfname = sprintf('data/pa1-%s-%s-calbody.txt', file_set, trial_letter);
cb = read_calbody(calbodyfname);

% Read calreadings file
calreadfname = sprintf('data/pa1-%s-%s-calreadings.txt', file_set, trial_letter);
cr = read_calreadings(calreadfname);

% Read EM pivot file.
em_fname = sprintf('data/pa1-%s-%s-empivot.txt', file_set, trial_letter);
em_data = read_empivot(em_fname);
em_data.letter = trial_letter;

% Read pivot data file.
opt_fname = sprintf('data/pa1-%s-%s-optpivot.txt', file_set, trial_letter);
opt_data = read_optpivot(opt_fname);
opt_data.letter = trial_letter;

if is_debug
    % Read the example output file, which we should be able to match
    outtruthfname = sprintf('data/pa1-%s-%s-output1.txt', file_set, trial_letter);
    outtruth = read_output(outtruthfname);
end

%% Run the calibration
% Prepare headers for output
out.NC = cr.NC;
out.Nframes = cr.Nframes;

% Preallocate some cell arrays.
Tac = cell(1, cr.Nframes);
Tad = cell(1, cr.Nframes);
Tdc_calc = cell(1, cr.Nframes);
c_err = nan(cr.NC, cr.Nframes);

for f = 1:numel(cr.frames)
    % Calculate FA: transform from optical sensor to calibration object.
    Tac{f} = correspondence_points(cb.a,  cr.frames{f}.a);
    reproj_in_a = dehomogenize(Tac{f} * homogenize(cb.a'));
    diffs = reproj_in_a - cr.frames{f}.a';
    pt_reproj_error = sum(sqrt(diffs.^2));
    assert(all(pt_reproj_error < 0.1, 'all'), 'Reprojection not very accurate');
    %fprintf('Norm reprojection error of Tac: %f\n', norm(pt_reproj_error));
    %plot_3d_axis_transform(Tac{f}, 'scale', 30);

    % Calculate FD: tranform from optical sensor to EM base.
    Tad{f} = correspondence_points(cb.d,  cr.frames{f}.d);
    reproj_in_a = dehomogenize(Tad{f} * homogenize(cb.d'));
    diffs = reproj_in_a - cr.frames{f}.d';
    pt_reproj_error = sum(sqrt(diffs.^2));
    assert(all(pt_reproj_error < 0.1, 'all'), 'Reprojection not very accurate');
    %fprintf('Norm reprojection error of Tad: %f\n', norm(pt_reproj_error));
    %plot_3d_axis_transform(Tcd{f}, 'scale', 30);

    % Combine FA and FD to estimate c
    Tdc_calc{f} = inv(Tad{f}) * Tac{f}; 
    out.frames{f}.C = dehomogenize(Tdc_calc{f} * homogenize(cb.c'))';

    if is_debug
        % Compare to the debug output data that we are trying to match
        c_err(:, f) = vecnorm(out.frames{f}.C - outtruth.frames{f}.C, 2, 2);
        fprintf('RMS distance from truth to our calc for points in frame: %.3f\n', rms(c_err(:, f)));
        fprintf('Worst point-point distance in frame: %.3f\n', max(c_err(:, f)));
    end
end

if is_debug
    worst_c_err = max(max(c_err));
    rms_c_err = rms(reshape(c_err, [], 1));
    fprintf('Point Calibration  Scenario %s   Worst c: %.3f   RMSE c: %.3f \n', trial_letter, worst_c_err, rms_c_err);
end

%% Run EM Pivot calibration

[em.btip, em.bpost, em.rmsPostError, em.worstDRFErr] = empivot(em_data);
out.empost = em.bpost';
if is_debug
    % Compare to the debug output data that we are trying to match
    em_post_err = norm(out.empost - outtruth.empost);
    fprintf('EM Pivot   Scenario %s   Post distance from truth: %.3f\n', trial_letter, em_post_err);
end

%% Run Optical Pivot calibration

[opt.btip, opt.bpost, opt.rmsPostError, opt.worstDRFErr] = optpivot(opt_data, cb.d);
out.optpost = opt.bpost';
if is_debug
    % Compare to the debug output data that we are trying to match
    opt_post_err = norm(out.optpost - outtruth.optpost);
    fprintf('Opt Pivot  Scenario %s   Post distance from truth: %.3f\n', trial_letter, opt_post_err);
end

%% Write the output file
% This can be compared to the debug output1 files provided in the
% assignment.

outfname = sprintf('data/pa1-%s-%s-outtest.txt', file_set, trial_letter);
write_output(outfname, out);


