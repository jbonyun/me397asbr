function q = rot2q(rot)
    % Converts a rotation matrix to quaternion representation.
    % Inputs:
    %   rot: 3x3 rotation matrix in SO(3)
    % Outputs:
    %   q: 4x1 quaternion in w;x;y;z form (scalar first)
    
   a0=1+rot(1,1)+rot(2,2)+rot(3,3);
   a1=1+rot(1,1)-rot(2,2)-rot(3,3);
   a2=1-rot(1,1)+rot(2,2)-rot(3,3);
   a3=1-rot(1,1)-rot(2,2)+rot(3,3);
   A=[a0,a1,a2,a3];
   if a0==max(A)
       q(1,1)=sqrt(a0)/2;
       q(2,1)=(rot(2,1)-rot(1,2))/(4*q(1,1));
       q(3,1)=(rot(1,3)-rot(3,1))/(4*q(1,1));
       q(4,1)=(rot(3,2)-rot(2,3))/(4*q(1,1));
   
   elseif a1==max(A)
       q(2,1)=sqrt(a1)/2;
       q(1,1)=(rot(3,2)-rot(2,3))/(4*q(2,1));
       q(3,1)=(rot(2,1)+rot(1,2))/(4*q(2,1));
       q(4,1)=(rot(3,1)+rot(1,3))/(4*q(2,1));

   elseif a2==max(A)
       q(3,1)=sqrt(a2)/2;
       q(1,1)=(rot(1,3)-rot(3,1))/(4*q(3,1));
       q(2,1)=(rot(2,1)+rot(1,2))/(4*q(3,1));
       q(4,1)=(rot(3,2)+rot(2,3))/(4*q(3,1));

   else
       q(4,1)=sqrt(a3)/2;
       q(1,1)=(rot(2,1)-rot(1,2))/(4*q(4,1));
       q(2,1)=(rot(3,1)+rot(1,2))/(4*q(4,1));
       q(3,1)=(rot(3,2)+rot(2,3))/(4*q(4,1));
   end