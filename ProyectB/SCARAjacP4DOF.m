function Jp = SCARAjacP4DOF(Q, Qp, L)

theta1 = Q(1);
theta2 = Q(2);
d3     = Q(3);
theta4 = Q(4);

theta1p = Qp(1);
theta2p = Qp(2);
d3p     = Qp(3);
theta4p = Qp(4);

l1 = L(1);
l2 = L(2);
l3 = L(3);

theta12    = theta1 + theta2;
theta12p   = theta1p + theta2p;

theta1234  = theta1 + theta2 + theta4;
theta1234p = theta1p + theta2p + theta4p;

Jp = zeros(4, 4);

% Derivative with respect to theta1
% J(1,1) = -l1*sin(theta1) - l2*sin(theta12) - l3*sin(theta1234)
Jp(1,1) = -l1*cos(theta1)*theta1p - l2*cos(theta12)*theta12p - l3*cos(theta1234)*theta1234p;
% J(2,1) =  l1*cos(theta1) + l2*cos(theta12) + l3*cos(theta1234)
Jp(2,1) = -l1*sin(theta1)*theta1p - l2*sin(theta12)*theta12p - l3*sin(theta1234)*theta1234p;
% J(3,1) = 0
Jp(3,1) = 0;
% J(4,1) = 0 (
Jp(4,1) = 0;

% Derivative with respect to theta2
% J(1,2) = -l2*sin(theta12) - l3*sin(theta1234)
Jp(1,2) = -l2*cos(theta12)*theta12p - l3*cos(theta1234)*theta1234p;
% J(2,2) =  l2*cos(theta12) + l3*cos(theta1234)
Jp(2,2) = -l2*sin(theta12)*theta12p - l3*sin(theta1234)*theta1234p;
% J(3,2) = 0
Jp(3,2) = 0;
% J(4,2) = 0 
Jp(4,2) = 0;

% Derivative with respect to d3
% J(1,3) = 0
Jp(1,3) = 0;
% J(2,3) = 0
Jp(2,3) = 0;
% J(3,3) = 0 
Jp(3,3) = 0;
% J(4,3) = 0

% Derivative with respect to theta4
% J(1,4) = -l3*sin(theta1234)
Jp(1,4) = -l3*cos(theta1234)*theta1234p;
% J(2,4) =  l3*cos(theta1234)
Jp(2,4) = -l3*sin(theta1234)*theta1234p;
% J(3,4) = 0
Jp(3,4) = 0;

Jp(4,4) = 0;

end
