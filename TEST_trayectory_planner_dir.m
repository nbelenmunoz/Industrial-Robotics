clc; clear all; close all;

l1 =100; l2 =70; % kinematic parameters (length of the links)
L=[l1; l2]';
Si=[100; -100]; % TCP position
Sf=[100; 100]; % TCP position
Ds = Sf-Si;

Qi = SCARAinv(Si,L,1); %Q of the beginning
Qf = SCARAinv(Sf,L,1); %Final position
dQ = Qf-Qi;

Fx=10; Fy=10;
Fs(1)=Fx;
Fs(2)=Fy;

Qia=Qi(1); dQa=dQ(1); %a is alpha
Qib=Qi(2); dQb=dQ(2); %b is beta

Six=Si(1); dSx=Ds(1);
Siy=Si(2); dSy=Ds(2);

tx1=3; tx2=6; tx3=9; %Distribution in terms of time
ty1=1; ty2=8; ty3=9;
t3 = tx3;

figure(1)
PlotAreaSCARA(L ,1); % draw the work area
hold on

i=1;
for t=0:0.1:t3
 % resx=Sshape(t,Six,dSx,tx1,tx2,tx3);
 % resy=Sshape(t,Siy,dSy,ty1,ty2,ty3);
 resa = Sshape(t, Qia, dQa, tx1, tx2, tx3);
 resb = Sshape(t, Qib, dQb, ty1, ty2, ty3);

 Q=[resa.pos; resb.pos];
 Qp=[resa.vel; resb.vel];
 Qpp=[resa.acc; resb.acc];

 % S = [resx.pos; resy.pos]; % Sp = [resx.vel;resy.vel]; % Spp=[resx.acc;resy.acc];
 S=SCARAdir(Q,L);
 

 % Q1=SCARAinv(S,L ,1);
 J = SCARAjac(Q, L);
 % Q1p=inv(J)*Sp;
 Jp=SCARAjacP(Q,Qp,L);
 % Q1pp=inv(J)*(Spp-Jp*Q1p);
 % 
 Fq(i,:)=-(J')*Fs';

 PlotScara(Q,L,'r',1)


time(i)=t;
px(i)=S(1);   %vx(i)=resx.vel;   ax(i)=resx.acc;
py(i)=S(2);   %vy(i)=resy.vel;   ay(i)=resy.acc;
q1(i)=Q(1); q1p(i)=Qp(1); q1pp(i)=Qpp(1);
q2(i)=Q(2); q2p(i)=Qp(2); q2pp(i)=Qpp(2);
i=i+1;
end

figure; plot(px, py); grid;
figure
subplot(2,2,1); plot(time, px); grid; % time, vx, time, ax
subplot(2,2,2); plot(time, py); grid; % time, vy, time, ay
subplot(2,2,3); plot(time, q1, time, q1p, time, q1pp); grid;
subplot(2,2,4); plot(time, q2, time, q2p, time, q2pp); grid;

figure;
plot(time, Fq(:,1), time, Fq(:,2)); grid %Kinematic solution
