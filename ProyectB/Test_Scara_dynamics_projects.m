clear all; close all; clc;

l1 = 0.33;  
l2 = 0.33;  
l3 = 0.33; 
g1 = 0.28; 
g2 = 0.283; 
g3 = 0.283; 


L = [l1; l2; l3; g1; g2; g3];
% M=diag(m,m,m,m3,m3,m3,(Jg1+Jg2+Jg3),mg2,mg2,mg2,(Jg1+Jg2),mg1,mg1,mg1,Jg1)
M = diag([0.5, 0.5, 0.5, 7, 7,7, 0.0565, 7.8, 7.8,7.8, 0.0465, 5, 5, 5, 0.0365]); 

Fse = zeros(15, 1);  
Fse(1) = 1;
Fse(2) = 1;
Fse(3) = 1;

Q = [pi/4, pi/3, pi/6, pi/8]; 
Qp = [-2.66; -2.2; 1.5; 1]; 
Qpp = [26.63; -12.36; 5.0; 1]; 

Sd = SCARAdirdin4DOF(Q, L);
J = SCARAjacdin4DOF(Q, L);
Jp = SCARAjacPdin4DOF(Q, Qp, L);

Spp = Jp * Qp + J * Qpp; 
Fsi = M * Spp; %Fsi = -diag(M) .* Spp;  

Fs = (Fse + Fsi); %force in the operational space
Fcq = -J' * Fs; % force in the joint space

% forces/torques
display(Fcq); 
