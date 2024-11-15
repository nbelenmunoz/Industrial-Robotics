clear all; close all; clc;

l1 = 0.33;  
l2 = 0.33;  
l3 = 0.33; 
g1 = 0.28; 
g2 = 0.283; 
g3 = 0.283; 


L = [l1; l2; l3; g1; g2; g3];

M = diag([0.5, 0.5, 7, 7, 0.0565, 7.8, 7.8, 0.0565]); 

Fse = zeros(8, 1);  
Fse(1) = 1;
Fse(2) = 1;

Q = [pi/4, pi/3, pi/6, pi/8]; 
Qp = [-2.66; -2.2; 1.5]; 
Qpp = [26.63; -12.36; 5.0]; 

Sd = SCARAdirdin4DOF(Q, L);
J = SCARAjacdin4DOF(Q, L);
Jp = SCARAjacPdin4DOF(Q, Qp, L);

Spp = Jp * Qp + J * Qpp; 
Fsi = -diag(M) .* Spp;  

Fs = (Fse + Fsi);
Fcq = -J' * Fs;

% forces/torques
display(Fcq); 
