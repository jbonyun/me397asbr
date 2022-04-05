function vector = trans2vector(trans)
    aa = rot2aa(trans(1:3,1:3));
    vector(1:3)=aa(1:3)*aa(4);
    vector(4:6)=trans(1:3,4);
    vector=vector';