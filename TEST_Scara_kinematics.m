% TEST SCARA
% script to test the direct and inverse kinematic problem: SCARA robot
clc; clear all
l1 =100; l2 =70; % kinematic parameters (length of the links)
L=[l1; l2]';
S=[100; -100]; % TCP position
figure(1)
Q1=SCARAinv(S,L ,1); % first solution (beta >0)
Q2=SCARAinv(S,L ,-1); % second solution( beta <0)
PlotAreaSCARA(L ,1); % draw the work area
PlotScara(Q1,L,'r',1) % draw the 1 sol. Scara
PlotScara(Q2,L,'b',1) % draw the 2 sol. Scara
hold off
S1=SCARAdir(Q1 ,L); % check 1 sol
S2=SCARAdir(Q2 ,L); % check 2 sol
