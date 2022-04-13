function [calread] = read_calreadings(fname)

    fid = fopen(fname);
    headers = textscan(fid, '%d, %d, %d, %d, %s', 1);
    fclose(fid);
    
    calread.ND = headers{1};
    calread.NA = headers{2};
    calread.NC = headers{3};
    calread.Nframes = headers{4};
    calread.fname = headers{5}{1};

    data = readmatrix(fname);
    
    curr_row = 1;
    for f = 1:calread.Nframes
        calread.frames{f}.d = data(curr_row:curr_row+calread.ND-1, :);
        curr_row = curr_row + calread.ND;
        calread.frames{f}.a = data(curr_row:curr_row+calread.NA-1, :);
        curr_row = curr_row + calread.NA;
        calread.frames{f}.c = data(curr_row:curr_row+calread.NC-1, :);
        curr_row = curr_row + calread.NC;
    end
    
    assert(curr_row == size(data,1)+1);