function [expon]=expm_sym(screw,angle)
    %W5-L1
    %W2-L2
    rot=eye(3,3)+skewsym_sym(screw(1:3))*sin(angle)+skewsym_sym(screw(1:3))^2*(1-cos(angle));
    trans=(eye(3,3)*angle+(1-cos(angle))*skewsym_sym(screw(1:3))+(angle-sin(angle))*skewsym_sym(screw(1:3))^2)*screw(4:6);
    expon=[rot trans;0 0 0 1];
