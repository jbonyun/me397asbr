%% Pivot calibration test

clear;

trial_letter = 'a';

% Read calbody file (for ground truth positions of markers d on EM base)
calbodyfname = sprintf('data/pa1-debug-%s-calbody.txt', trial_letter);
cb = read_calbody(calbodyfname);

% Read pivot data file.
opt_fname = sprintf('data/pa1-debug-%s-optpivot.txt', trial_letter);
data = read_optpivot(opt_fname);
data.letter = trial_letter;

% Use the function to do the work.
optpivot(data, cb.d);
