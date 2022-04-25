function write_output(fname, out)
    % Inputs:
    %    fname: filename to write to, char array
    %    out:   struct to write to the file, with fields:
    %             out.NC
    %             out.Nframes
    %             out.frames{}.C

    fid = fopen(fname, 'w');

    % Header
    [~, fjustname, fext] = fileparts(fname);
    fbasename = strcat(fjustname, fext);
    fprintf(fid, '%d, %d, %s\n', out.NC, out.Nframes, fbasename);

    % EM Post
    if isfield(out, 'empost')
        fprintf(fid, '%8.2f, %8.2f, %8.2f\n', out.empost(1), out.empost(2), out.empost(3));
    else
        fprintf(fid, '%f, %f, %f\n', nan, nan, nan);
    end

    % Opt Post
    if isfield(out, 'optpost')
        fprintf(fid, '%8.2f, %8.2f, %8.2f\n', out.optpost(1), out.optpost(2), out.optpost(3));
    else
        fprintf(fid, '%f, %f, %f\n', nan, nan, nan);
    end
    
    % Frames
    for f = 1:out.Nframes
        for ic = 1:out.NC
            fprintf(fid, '%8.2f, %8.2f, %8.2f\n', out.frames{f}.C(ic, 1), out.frames{f}.C(ic, 2), out.frames{f}.C(ic, 3));
        end
    end
    
    fclose(fid);