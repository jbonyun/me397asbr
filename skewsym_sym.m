function ss = skewsym_sym(vec)

        ss = sym('a%d%d',[3,3]);
        ss(1, 1) = 0;
        ss(1, 2) = -vec(3);
        ss(1, 3) =  vec(2);
        ss(2, 1) = vec(3);
        ss(2, 2) = 0;
        ss(2, 3) =  -vec(1);
        ss(3, 1) = -vec(2);
        ss(3, 2) =vec(1);
        ss(3, 3) =  0;
