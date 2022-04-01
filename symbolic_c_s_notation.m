function [out] = symbolic_c_s_notation(insymbolic)
    out = string(insymbolic);
    for i = 1:7
        out = strrep(strrep(out, sprintf('cos(theta%d)',i), sprintf('c%d',i)), sprintf('sin(theta%d)',i), sprintf('s%d',i));
    end