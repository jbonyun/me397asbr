%%
clear;

%% Read calbody file
calbodyfname = 'data/pa1-debug-a-calbody.txt';
cb = read_calbody(calbodyfname);

%% Read calreadings file
calreadfname = 'data/pa1-debug-a-calreadings.txt';
cr = read_calreadings(calreadfname);
%% Read empivot file
empivfname = 'data/pa1-debug-a-empivot.txt';
ep = read_empivot(empivfname);

%% Read optpivot file
optpivfname = 'data/pa1-debug-a-optpivot.txt';
op = read_optpivot(optpivfname);

%% Read output file
outfname = 'data/pa1-debug-a-output1.txt';
out = read_output(outfname);

%% TODO: test writing output file