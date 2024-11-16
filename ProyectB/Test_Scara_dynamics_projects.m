clear all; close all; clc;

l1 = 0.33;  
l2 = 0.33;  
l3 = 0.33; 

L = [l1; l2; l3];

m_x = 0.5;       
m_y = 0.5;      
m_z = 7.0;     
I_theta = 0.0565; 

M = diag([m_x, m_y, m_z, I_theta]); 

Fse = zeros(4, 1);  
Fse(1) = 1;  
Fse(2) = 1;  

Q = [pi/4; pi/3; pi/6; pi/8]; 
Qp = [-2.66; -2.2; 1.5; 0.5];     
Qpp = [26.63; -12.36; 5.0; 1.0];  

Sd = SCARAdirdin4DOF(Q, L);      
J = SCARAjacdin4DOF(Q, L);       
Jp = SCARAjacPdin4DOF(Q, Qp, L); 

Spp = Jp * Qp + J * Qpp; 

Fsi = -M * Spp;  

Fs = Fse + Fsi;

Fcq = -J' * Fs;

disp('Forces/Torques at the joints:');
disp(Fcq); 
