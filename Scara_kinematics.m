clear all
clc
close all

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

%% Scara position function
function S= SCARAdir(Q,L) %Q position and L the vector with the characteristics of the robot
% direct kinematics: SCARA robot
l1=L(1);
l2=L(2);
S= zeros(2,1);
S(1)= l1*cos(Q(1))+l2*cos(Q(1)+Q(2));
S(2)= l1*sin(Q(1))+l2*sin(Q(1)+Q(2));
end

%% Scara Jacobian
function J = SCARAjac(Q,L)
% Jacobian matrix: SCARA robot
l1=L(1);
l2=L(2);
J=zeros(2,2);
J(1,1)= -l1*sin(Q(1))-l2*sin(Q(1)+Q(2)); J(1,2)= - l2*sin(Q(1)+Q(2)); %Definition of jacobian matrix
J(2,1)= l1*cos(Q(1))+l2*cos(Q(1)+Q(2)); J(2,2)= l2*cos(Q(1)+Q(2));
end

%% Derivative of J
function Jp = SCARAjacP(Q,Qp,L) %Qp is velocity or the derivative. p means dot
% time derivative of the Jacobian matrix: SCARA robot
l1=L(1);
l2=L(2);
Jp= zeros(2,2);
Jp(1,1)= -l1*cos(Q(1))*Qp(1)- l2*cos(Q(1)+Q(2))*(Qp(1)+Qp(2));
Jp(1,2)= -l2*cos(Q(1)+Q(2))*(Qp(1)+Qp(2));
Jp(2,1)= -l1*sin(Q(1))*Qp(1)-l2*sin(Q(1)+Q (2))*(Qp(1)+Qp(2));
Jp(2,2)= -l2*sin(Q(1)+Q(2))*(Qp(1)+Qp(2));
end

%% Inverse kinematic: SCARA robot
function Q= SCARAinv(S,L,sol ) %s end effector position, L dimension, sol solution we want to use
Q= zeros(2,1);
x=S(1); y=S(2);
l1=L(1); l2=L(2);
beta=acos((x^2+ y^2-l1^2 -l2^2)/(2*l1*l2));
if (sol >0)
 Q(2)= beta;
else
 Q(2)= - beta;
end
Q(1)= atan2(y,x)- atan2(l2*sin(Q(2)), l1+l2*cos(Q(2)));
end

%% Scara plot
function PlotScara (Q,L,colore,fig)
figure(fig)
q1=Q(1);
q2=Q(2);
l1=L(1);
l2=L(2);
x1=l1*cos(q1);
y1=l1*sin(q1);
x2=x1+l2*cos(q1+q2);
y2=y1+l2*sin(q1+q2);
plot([0 x1 x2],[0 y1 y2],'LineWidth',2,'color',colore);
axis equal;
app=(l1+l2 )*1.1;
xlim([-app app]);
ylim([-app app]);
end

%% Plot area of scara
function PlotAreaSCARA(L,fig)
figure(fig); hold off; plot ([0],[0]); hold on; grid on;
l1=L(1); l2=L(2);
d1=l1+l2; d2=abs(l1 -l2);
nn =50;
for i=0: nn
fi =2*pi/nn*i;
x1=l1*cos(fi); y1=l1*sin(fi);
xa=d1*cos(fi); ya=d1*sin(fi);
xb=d2*cos(fi); yb=d2*sin(fi);
 if i~=0
plot([x1old x1],[y1old y1],'LineWidth',1,'color','green');
 plot([xaold xa],[yaold ya],'LineWidth',2,'color','green');
 plot([xbold xb],[ybold yb],'LineWidth',2,'color','green');
 end
 x1old =x1; y1old =y1;
 xaold =xa; yaold =ya;
 xbold =xb; ybold =yb;
end
hold on
end
