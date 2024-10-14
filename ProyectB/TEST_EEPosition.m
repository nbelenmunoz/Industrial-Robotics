% TEST SCARA 4DOF
% Script to test the direct and inverse kinematic problem for a SCARA robot
clc; clear all; close all;

l1 = 300;  
l2 = 200;  
l3 = 160;  
L = [l1; l2; l3];  

% Target position for the end effector (X, Y, Z) and orientation phi
X_target = -600;        
Y_target = -10;          
Z_target = 150;          
phi_target = pi;    

% End-effector position
S = [X_target; Y_target; Z_target; phi_target];


Q1 = SCARAinv3DOF(S, L, 1);   
Q2 = SCARAinv3DOF(S, L, -1);  

% Plot the work area and robot configurations
figure(1);
PlotAreaSCARA3DOF(L, 1);           
hold on
PlotScara3DOF(Q1(1:3), L, 'r', 1); 
PlotScara3DOF(Q2(1:3), L, 'b', 1); 
hold on

% Check the forward kinematics
S1 = SCARAdir3DOF(Q1, L);  % End-effector position from first solution
S2 = SCARAdir3DOF(Q2, L);  % End-effector position from second solution


