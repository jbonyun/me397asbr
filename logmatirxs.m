function [screw, angle]=logmatirxs(trans)
	     %W2-L2  PAGE8
         %w5-L2 page2
         %logarithm of tranformation
         %input tranformation matrix
         %output screw axis and angle

         % TODO: must handle R==eye! W5-L2 p2
         % TODO: replace with trans2screw, which does the same thing but is
         %       part of our tested set of functions.

         %matirx logarithm of rotations
         rot=trans(1:3,1:3);
         angle=acos((trace(rot)-1)/2);
         w_skew=(rot-rot')/(2*sin(angle));
         w=unskewsym(w_skew);
         G=eye(3,3)/angle-1/2*w_skew+(1/angle-1/2*cot(angle/2))*w_skew^2;
         v=G*trans(1:3,4);
         screw(1:3)=w;
         screw(4:6)=v;
         screw=screw';


