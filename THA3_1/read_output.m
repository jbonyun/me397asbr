function [out] = read_output(fname)

    fid = fopen(fname);
    headers = textscan(fid, '%d, %d, %s', 1);
    emread = textscan(fid, '%f, %f, %f', 1);
    opread = textscan(fid, '%f, %f, %f', 1);
    fclose(fid);
    
    out.NC = headers{1};
    out.Nframes = headers{end-1};
    out.fname = headers{end}{1};

    out.empost = cell2mat(emread);
    out.optpost = cell2mat(opread);

    data = readmatrix(fname, 'Range', 4);
    
    curr_row = 1;
    for f = 1:out.Nframes
        out.frames{f}.C = data(curr_row:curr_row+out.NC-1, :);
        curr_row = curr_row + out.NC;
    end
    
    assert(curr_row == size(data,1)+1, sprintf('File had %d rows and we expected %d', size(data,1)+1, out.Nframes * (out.NC) + 1 + 2));