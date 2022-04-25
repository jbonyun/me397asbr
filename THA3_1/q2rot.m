function rot=q2rot(q)
%input: unit quaternion 1by4
%output: rotation matrix 3by3

rot(1,1)=q(1)^2+q(2)^2-q(3)^2-q(4)^2;
rot(1,2)=2*(q(2)*q(3)-q(1)*q(4));
rot(1,3)=2*(q(2)*q(4)+q(1)*q(3));

rot(2,1)=2*(q(2)*q(3)+q(1)*q(4));
rot(2,2)=q(1)^2-q(2)^2+q(3)^2-q(4)^2;
rot(2,3)=2*(q(3)*q(4)-q(1)*q(2));

rot(3,1)=2*(q(2)*q(4)-q(1)*q(3));
rot(3,2)=2*(q(3)*q(4)+q(1)*q(2));
rot(3,3)=q(1)^2-q(2)^2-q(3)^2+q(4)^2;