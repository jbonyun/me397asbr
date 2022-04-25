function M=get_M(qa,qb)
%input: unit quaternion 4by1 qa,qb
%output: M matrix 4by4

M(1,1)=qa(1)-qb(1);
M(2,2:4)=-transpose(qa(2:4)-qb(2:4));
M(2:4,1)=qa(2:4)-qb(2:4);
M(2:4,2:4)=(qa(1)-qb(1))*eye(3,3)+skewsym(qa(2:4)+qb(2:4));
