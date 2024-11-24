clc; clear all

%% l1 =100; l2 =70;  % kinematic parameters (length of the links)
l1 = 300;  
l2 = 200;  
l3 = 160;
L=[l1; l2; l3]';
Si=[100; -100; 5]; % TCP position(Si can be the initial position) %S=[170; 0] this represent the singularity configuration
Sf=[100; 100; 20]; %end effector final position
Ds=Sf-Si %difference between final and initial position

%to plan the trajectory in the joint space
Qi=SCARAinv4DOF(Si,L,1); %initial position in the joint space
Qf=SCARAinv4DOF(Sf,L,1); %final position in the joint space
dQ=Qf-Qi;


Fx=10; Fy=10; Fz=10; 
Fs(1)=Fx;
Fs(2)=Fy;
Fs(3)=Fz;

Qia=Qi(1); dQa=dQ(1); 
Qib=Qi(2); dQb=dQ(2);

Six=Si(1); Dsx=Ds(1); 
Siy=Si(2); Dsy=Ds(2);
Siz=Si(3); Dsz=Ds(3);

%t1=3 ; t2=6; t3=9; % here we use the same time for motion curve along x and y
%when the line are the same for the 2 motions we expect a line but when
%they are different as in the line below
% t1=positive acceleration,t2=zero acceleration,t3=decceleration
tx1=3 ; tx2=6; tx3=9;
ty1=1 ; ty2=8; ty3=9;
tz1=1 ; tz2=4; tz3=7;

t3=tx3;

figure(1)
%PlotAreaSCARA(L ,1); % draw the work area
i=1;
for t=0:0.1:t3 %here we use a sample time of 0,1 seconde and the sample time most be the same for all the function
 resx=Sshape4DOF(t,Six,Dsx,tx1,tx2,tx3); %behaviour of the motion curve along x
 resy=Sshape4DOF(t,Siy,Dsy,ty1,ty2,ty3); %behaviour of the motion curve along y
 resz=Sshape4DOF(t,Siz,Dsz,tz1,tz2,tz3)

 resa=Sshape4DOF(t,Qia,dQa,tx1,tx2,tx3);
 resb=Sshape4DOF(t,Qib,dQb,ty1,ty2,ty3);
 resc=Sshape4DOF(t,Qib,dQb,ty1,ty2,ty3);

 Q=[resa.pos;resb.pos;resc.pos];
 Qp=[resa.vel;resb.vel;resc.vel];
 Qpp=[resa.acc;resb.acc;resc.acc];

 % S= [resx.pos;resy.pos;resz.pos]; %inother to compose the new value of S
 % Sp= [resx.vel;resy.vel];
 % Spp= [resx.acc;resy.acc];

 S=SCARAdir4DOF(Q,L); %position of the end effector

% Q1=SCARAinv(S,L ,1); %to calculate q1 and q2 which are alpha and beta
 J=SCARAjac4DOF(Q,L);
%  Q1p=inv(J)*Sp ;%to calculate the joint joint speed
 Jp=SCARAjacP4DOF(Q,Qp,L);
%  Q1pp=inv(J)*(Spp-Jp*Q1p) ; %to calculate the joint acceleration
 Fq(i,:)=-(J')*Fs'; %torque apply in the joint space

 PlotScara4DOF(Q,L,'r',1) % plot position time after time

 time(i)=t; % i save the variable time in the time and i need a counter because we are within a for so i need to add element to the vector time
 px(i)=S(1); %vx(i)=resx.vel; ax(i)=resx.acc; 
 py(i)=S(2); %vy(i)=resy.vel; ay(i)=resy.acc;
 pz(i)=S(3);
 q1(i)=Q(1);q1p(i)=Qp(1);q1pp(i)=Qpp(1);%in vector q1 that is alpha i place the value Q1(in the first element)
 q2(i)=Q(2);q2p(i)=Qp(2);q2pp(i)=Qpp(2) ;%beta
 q3(i)=Q(3);q3p(3)=Qp(3);q3pp(i)=Qpp(3) ; %gamma

 % v(i)=res.vel;
 % a(i)=res.acc; 
 i=i+1;
end

figure;plot(px,py,pz);grid; %this is the trajectory of my end effector in the working space
figure
subplot(2,2,1);plot(time,px);grid; %,time,vx,time,ax);grid;
subplot(2,2,2);plot(time,py);grid; %,time,vy,time,ay);grid;
subplot(2,2,3);plot(time,pz);grid; %,time,vz,time,az);grid;
subplot(2,2,4);plot(time,q1,time,q1p,time,q1pp);grid; %plot of deplscement,velocity and acceleration of the joint space
subplot(2,2,5);plot(time,q2,time,q2p,time,q2pp);grid;
subplot(2,2,6);plot(time,q3,time,q3p,time,q3pp);grid;


 figure;
  plot(time,Fq(:,1),time,Fq(:,2)),time,Fq(:,3);grid %plot wrt time the torque apply in q1 and q2
% figure(1)
% Q1=SCARAinv(S,L ,1); % first solution (beta >0)
% Q2=SCARAinv(S,L ,-1); % second solution( beta <0)
% PlotAreaSCARA(L ,1); % draw the work area
% PlotScara(Q1,L,'r',1) % draw the 1 sol. Scara  
% PlotScara(Q2,L,'b',1) % draw the 2 sol. Scara
% hold off
% S1=SCARAdir(Q1 ,L); % check 1 sol, s1 and s2 here are the evaluation of s for a given value of Q
% S2=SCARAdir(Q2 ,L); % check 2 sol we evaluate the position knowing the
% angle alpha and beta
