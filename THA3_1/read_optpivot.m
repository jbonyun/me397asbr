function [op] = read_optpivot(fname)

    fid = fopen(fname);
    headers = textscan(fid, '%d, %d, %d, %s', 1);
    fclose(fid);
    
    op.ND = headers{1};
    op.NH = headers{2};
    op.Nframes = headers{end-1};
    op.fname = headers{end}{1};

    data = readmatrix(fname, 'Range', 2);
    
    curr_row = 1;
    for f = 1:op.Nframes
        op.frames{f}.D = data(curr_row:curr_row+op.ND-1, :);
        curr_row = curr_row + op.ND;
        op.frames{f}.H = data(curr_row:curr_row+op.NH-1, :);
        curr_row = curr_row + op.NH;
    end
    
    assert(curr_row == size(data,1)+1, sprintf('File had %d rows and we expected %d', size(data,1)+1, op.Nframes * (op.ND + op.NH) + 1));