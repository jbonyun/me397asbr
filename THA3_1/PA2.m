clear;
[q_Robot_config, q_camera_config,t_Robot_config,t_camera_config ]=data_quaternion();

% get E and S matrix of each frame from quaternion
for i=1:10
    Rot_E=q2rot(q_Robot_config(i,:));
    p_E=transpose(t_Robot_config(i,:));
    E(:,:,i) =rottranslation2trans(Rot_E,p_E);

    Rot_S=q2rot(q_camera_config(i,:));
    p_S=transpose(t_camera_config(i,:));
    S(:,:,i) =rottranslation2trans(Rot_S,p_S);

end

% get A and B matrix from S and E
for i=1:9
    A(:,:,i) = inv(E(:,:,i))*E(:,:,i+1);
    B(:,:,i) = S(:,:,i)*inv(S(:,:,i+1));
    RA=trans2rot(A(:,:,i));
    RB=trans2rot(B(:,:,i));
    q_A(:,i)=rot2q(RA);
    q_B(:,i)=rot2q(RB);
    M(4*i-3:4*i,:)=get_M(q_A(:,i),q_B(:,i));
end
% use SVD method to get quarternion
[U,S0,V] = svd(M);
Rx=q2rot(V(:,4));

for i=1:9
    RA=trans2rot(A(:,:,i));
    pA= trans2translation(A(:,:,i));
    pB=trans2translation(B(:,:,i));
    A_2(3*i-2:3*i,:)=RA-eye(3,3);
    B_2(3*i-2:3*i,1)=Rx*pB-pA;
end
% use least square method to get px
px=inv(transpose(A_2)*A_2)*transpose(A_2)*B_2;
X=[Rx,px;0 0 0 1]


%% use 10 noisy data
clear;
[q_Robot_config, q_camera_config,t_Robot_config,t_camera_config ]=data_quaternion_noisy();
for i=1:10
    Rot_E=q2rot(q_Robot_config(i,:));
    p_E=transpose(t_Robot_config(i,:));
    E(:,:,i) =rottranslation2trans(Rot_E,p_E);

    Rot_S=q2rot(q_camera_config(i,:));
    p_S=transpose(t_camera_config(i,:));
    S(:,:,i) =rottranslation2trans(Rot_S,p_S);



end

for i=1:9
    A(:,:,i) = inv(E(:,:,i))*E(:,:,i+1);
    B(:,:,i) = S(:,:,i)*inv(S(:,:,i+1));
    RA=trans2rot(A(:,:,i));
    RB=trans2rot(B(:,:,i));
    q_A(:,i)=rot2q(RA);
    q_B(:,i)=rot2q(RB);
    M(4*i-3:4*i,:)=get_M(q_A(:,i),q_B(:,i));

end
[U,S0,V] = svd(M);
Rx=q2rot(V(:,4));

for i=1:9
    RA=trans2rot(A(:,:,i));
    pA= trans2translation(A(:,:,i));
    pB=trans2translation(B(:,:,i));
    A_2(3*i-2:3*i,:)=RA-eye(3,3);
    B_2(3*i-2:3*i,1)=Rx*pB-pA;
end

px=inv(transpose(A_2)*A_2)*transpose(A_2)*B_2;
X=[Rx,px;0 0 0 1]
%% use 5 noisy data
clear;
[q_Robot_config, q_camera_config,t_Robot_config,t_camera_config ]=data_quaternion_noisy();
for i=1:5
    Rot_E=q2rot(q_Robot_config(i,:));
    p_E=transpose(t_Robot_config(i,:));
    E(:,:,i) =rottranslation2trans(Rot_E,p_E);

    Rot_S=q2rot(q_camera_config(i,:));
    p_S=transpose(t_camera_config(i,:));
    S(:,:,i) =rottranslation2trans(Rot_S,p_S);



end

for i=1:4
    A(:,:,i) = inv(E(:,:,i))*E(:,:,i+1);
    B(:,:,i) = S(:,:,i)*inv(S(:,:,i+1));
    RA=trans2rot(A(:,:,i));
    RB=trans2rot(B(:,:,i));
    q_A(:,i)=rot2q(RA);
    q_B(:,i)=rot2q(RB);
    M(4*i-3:4*i,:)=get_M(q_A(:,i),q_B(:,i));

end
[U,S0,V] = svd(M);
Rx=q2rot(V(:,4));

for i=1:4
    RA=trans2rot(A(:,:,i));
    pA= trans2translation(A(:,:,i));
    pB=trans2translation(B(:,:,i));
    A_2(3*i-2:3*i,:)=RA-eye(3,3);
    B_2(3*i-2:3*i,1)=Rx*pB-pA;
end

px=inv(transpose(A_2)*A_2)*transpose(A_2)*B_2;
X=[Rx,px;0 0 0 1]
