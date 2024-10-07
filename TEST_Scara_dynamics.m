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

Q=[-0.244;2.375];
Qp=[-2.66;-2.2];
Qpp=[26.63;-12.36];

%First Request: Evaluate the value of Fq (More details in the ppt) 

%position of the extended 2R(operational space)
Sd=SCARAdirdin(Q,L);
%Jacobians matrix and its time derivative
J=SCARAjacdin(Q,L);
Jp=SCARAjacPdin(Q,Qp,L);
% acceleration of the extended 2R
Spp=Jp*Qp+J*Qpp;
%inertial forces
Fsi=-M*Spp;
% forces in the operational space
Fs=(Fse+Fsi);
% forces in the joint space
Fcq=-J'*Fs;
display(Fcq);

%Second request:Evaluate the motor torques when the EE is moving along a linear trajectory (More details in the ppt) 
