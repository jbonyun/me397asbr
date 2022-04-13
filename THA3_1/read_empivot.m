function [emp] = read_empivot(fname)

    fid = fopen(fname);
    headers = textscan(fid, '%d, %d, %s', 1);
    fclose(fid);

    emp.NG = headers{1};
    emp.Nframes = headers{2};
    emp.fname = headers{3}{1};

    data = readmatrix(fname, 'Range', 2);

    curr_row = 1;
    for f = 1:emp.Nframes
        emp.frames{f}.G = data(curr_row:curr_row+emp.NG-1, :);
        curr_row = curr_row + emp.NG;
    end

    assert(curr_row == size(data,1)+1, sprintf('File had %d rows and we expected %d', size(data,1)+1, emp.Nframes * (emp.NG) + 1));