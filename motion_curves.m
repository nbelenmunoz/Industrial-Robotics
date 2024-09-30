clear all; close all; clc;
S0=1; dS=1;
t1=3; t2=6; t3=9;
i=1;
for t=0:0.1:t3
 res=Sshape(t,S0,dS,t1,t2,t3);
 time(i)=t;
 p(i)=res.pos;
 …
 i=i+1;
end

%% Motion curves
function res=Sshape(t,S0,dS,t1,t2,t3)
V=(dS)/(t1/2 + (t2-t1) + (t3-t2)/2);
A=V/t1;
D=V/(t3-t2);
if t<t1
 res.pos=S0+1/2*A*t^2;
 res.vel=A*t;
 res.acc=A;
elseif t<t2
 res.pos=S0+1/2*A*t1^2+V*(t-t1);
 res.vel=V;
 res.acc=0;
else
 res.pos=S0+dS-D*(t3-t)^2/2;
 res.vel=V-D*(t-t2);
 res.acc=-D;
end
end
