clc; clear all; close all;

l1 = 300;  
l2 = 200;  
l3 = 160;  
L = [l1; l2; l3];  

X_target = -600;        
Y_target = -10;          
Z_target = 150;          
phi_target = pi;    

S = [X_target; Y_target; Z_target; phi_target];

Q1 = SCARAinv4DOF(S, L, 1);    
Q2 = SCARAinv4DOF(S, L, -1);   


figure(1);
PlotAreaSCARA4DOF(L, 1);           
hold on;
PlotScara4DOF(Q1, L, 'r', 1);      
PlotScara4DOF(Q2, L, 'b', 1);      
hold off;

S1 = SCARAdir4DOF(Q1, L); 
S2 = SCARAdir4DOF(Q2, L);  

% Display the joint variables and end-effector positions
disp('First solution joint variables Q1 (theta1, theta2, d3, theta4):');
disp(Q1);

disp('Second solution joint variables Q2 (theta1, theta2, d3, theta4):');
disp(Q2);

disp('End-effector position from forward kinematics S1 (X, Y, Z, phi):');
disp(S1);

disp('End-effector position from forward kinematics S2 (X, Y, Z, phi):');
disp(S2);
