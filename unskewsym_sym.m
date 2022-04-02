function vector=unskewsym_sym(skew)
        vector = sym('a%d%d',[3,1]);
        vector(1)=skew(3,2);
        vector(2)=skew(1,3);
         vector(3)=skew(2,1);