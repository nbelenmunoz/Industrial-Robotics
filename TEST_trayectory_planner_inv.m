clc; clear all; close all;

l1 =100; l2 =70; % kinematic parameters (length of the links)
L=[l1; l2]';
Si=[100; -100]; % TCP position
Sf=[100; 100]; % TCP position
Ds = Sf-Si;

Fx=10; Fy=10;
Fs(1)=Fx;
Fs(2)=Fy;

Six=Si(1); dSx=Ds(1);
Siy=Si(2); dSy=Ds(2);

tx1=3; tx2=6; tx3=9;
ty1=1; ty2=8; ty3=9;
t3 = tx3;

figure(1)
PlotAreaSCARA(L ,1); % draw the work area
hold on

i=1;
for t=0:0.1:t3
 resx=Sshape(t,Six,dSx,tx1,tx2,tx3);
 resy=Sshape(t,Siy,dSy,ty1,ty2,ty3);
 
 S = [resx.pos; resy.pos];
 Sp = [resx.vel;resy.vel];
 Spp=[resx.acc;resy.acc];
 Q1=SCARAinv(S,L ,1);
 J = SCARAjac(Q1, L);
 Q1p=inv(J)*Sp;
 Jp=SCARAjacP(Q1,Q1p,L);
 Q1pp=inv(J)*(Spp-Jp*Q1p);

 Fq(i,:)=-(J')*Fs';

 PlotScara(Q1,L,'r',1)


time(i)=t;
px(i)=resx.pos;   vx(i)=resx.vel;   ax(i)=resx.acc;
py(i)=resy.pos;   vy(i)=resy.vel;   ay(i)=resy.acc;
q1(i)=Q1(1); q1p(i)=Q1p(1); q1pp(i)=Q1pp(1);
q2(i)=Q1(2); q2p(i)=Q1p(2); q2pp(i)=Q1pp(2);
i=i+1;
end

figure; plot(px, py); grid;
figure
subplot(2,2,1); plot(time, px, time, vx, time, ax); grid;
subplot(2,2,2); plot(time, py, time, vy, time, ay); grid;
subplot(2,2,3); plot(time, q1, time, q1p, time, q1pp); grid;
subplot(2,2,4); plot(time, q2, time, q2p, time, q2pp); grid;

figure;
plot(time, Fq(:,1), time, Fq(:,2)); grid %Kinematic solution
