%Second Request
clear all; close all; clc;
% robot parameters
l1=0.33; %[m]
l2=0.33; %[m]
g1=0.28; %[m]
g2=0.283; %[m]

L=[l1;l2;g1;g2]; %[m]

M=diag([0.5,0.5,7,7,0.0565,7.8,7.8,0.0565]); %Masses of our robot according to slide 3

Fse=zeros(8,1);
Fse(1)=1;
Fse(2)=1;

S0=[0.2;0.5]; % gripper position
Sf=[ -0.5; 0.2];
dS=Sf-S0;

dT =1/100;
T=5;
tt =[0:dT:T]; %Time used
n= length(tt); %Number of T elements

for i=1:n
    resx=cycloidal(tt(i),T,S0(1),dS(1));
    resy=cycloidal(tt(i),T,S0(2),dS(2));

    S=[resx.pos;resy.pos];
    Sp=[resx.vel;resy.vel];
    Spp=[resx.acc;resy.acc];

    Q=SCARAinv(S,L,1); %Having S you can calculate Q!
    J=SCARAjac(Q,L);
    Qp=inv(J)*Sp;
    Jp=SCARAjacP(Q,Qp,L);
    Qpp=inv(J)*(Spp-Jp*Qp);    
end


Q=[-0.244;2.375];
Qp=[-2.66;-2.2];
Qpp=[26.63;-12.36];

