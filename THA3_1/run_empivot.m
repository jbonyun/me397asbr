%% Pivot calibration on EM pointer.

clear;

trial_letter = 'a';  % 'a' through 'g' exist

% Read pivot data file.
em_fname = sprintf('data/pa1-debug-%s-empivot.txt', trial_letter);
data = read_empivot(em_fname);
data.letter = trial_letter;

% Use the function to do the work.
empivot(data);
