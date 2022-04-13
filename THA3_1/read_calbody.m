function [calbody] = read_calbody(fname)

    fid = fopen(fname);
    headers = textscan(fid, '%d, %d, %d, %s', 1);
    fclose(fid);
    
    calbody.ND = headers{1};
    calbody.NA = headers{2};
    calbody.NC = headers{3};
    calbody.fname = headers{4};

    options = detectImportOptions(fname);
    options.VariableNames = {'x','y','z'};
    data = readmatrix(fname, options);    
    
    calbody.d = data(1:calbody.ND, :);
    calbody.a = data(calbody.ND+1:calbody.ND+calbody.NA, :);
    calbody.c = data(calbody.ND+calbody.NA+1:end, :);
    