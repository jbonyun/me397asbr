function expon=expm_sym(screw,angle)
    %W5-L1 page8
    %W2-L2  page11
    %input screw 6*1 angle scaler
    %output expon transformation matirx 4*4
    rot=eye(3,3)+skewsym_sym(screw(1:3))*sin(angle)+skewsym_sym(screw(1:3))^2*(1-cos(angle));
    G=angle*eye(3,3)+(1-cos(angle))*skewsym_sym(screw(1:3))+(angle-sin(angle))*skewsym_sym(screw(1:3))^2;
    v=G*screw(4:6);
    expon=[rot v;0 0 0 1];
