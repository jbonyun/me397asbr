%% The "debug" letters

letters = 'abcdefg'; for li = 1:numel(letters); trial_letter = letters(li); run_calibration; end;

%% The "unknown" letters

letters = 'hijk'; for li = 1:numel(letters); trial_letter = letters(li); run_calibration; end;