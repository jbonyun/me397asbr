function ad=adjoint_sym(trans)

	    ad =sym('a%d%d',[6,6]);
        ad = [trans(1:3,1:3) zeros(3,3); skewsym_sym(trans(1:3,4)) * trans(1:3,1:3) trans(1:3,1:3)];
