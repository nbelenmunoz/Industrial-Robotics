function Jp = SCARAjacP4DOF(Q, Qp, L)

theta1 = Q(1);
theta2 = Q(2);
theta3 = Q(3);
d4 = Q(4);

theta1p = Qp(1);
theta2p = Qp(2);
theta3p = Qp(3);
d4p     = Qp(4);


l1 = L(1);
l2 = L(2);
l3 = L(3);

theta12    = theta1 + theta2;
theta12p   = theta1p + theta2p;

theta123  = theta1 + theta2 + theta3;
theta123p = theta1p + theta2p + theta3p;

Jp = zeros(3, 4);

% Derivative with respect to theta1
Jp(1,1) = -l1*cos(theta1)*theta1p - l2*cos(theta12)*theta12p - l3*cos(theta123)*theta123p;
Jp(1,2) = -l2*cos(theta12)*theta12p - l3*cos(theta123)*theta123p;
Jp(1,3) = -l3*cos(theta123)*theta123p;
Jp(1,4) = 0;

Jp(2,1) = -l1*sin(theta1)*theta1p - l2*sin(theta12)*theta12p - l3*sin(theta123)*theta123p;
Jp(2,2) = -l2*sin(theta12)*theta12p - l3*sin(theta123)*theta123p;
Jp(2,3) = -l3*sin(theta123)*theta123p;
Jp(2,4) = 0;

Jp(3,1) = 0;
Jp(3,2) = 0;
Jp(3,3) = 0;
Jp(3,4) = 0;

end
