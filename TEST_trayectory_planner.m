clc; clear all; close all;

l1 =100; l2 =70; % kinematic parameters (length of the links)
L=[l1; l2]';
Si=[100; -100]; % TCP position
Sf=[100; 100]; % TCP position
Ds = Sf-Si;

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
 Q1=SCARAinv(S,L ,1);
 PlotScara(Q1,L,'r',1)


 time(i)=t;
 px(i)=resx.pos;
 py(i)=resy.pos;
 q1(i)=Q1(1);
 q2(i)=Q1(2);

 i=i+1;
end

figure; plot(px, py); grid;
figure
subplot(2,2,1);plot(time, px);grid;
subplot(2,2,2);plot(time, py);grid;
subplot(2,2,3);plot(time, q1);grid;
subplot(2,2,4);plot(time, q2);grid;




