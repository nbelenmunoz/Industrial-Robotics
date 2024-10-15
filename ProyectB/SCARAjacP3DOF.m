function Jp = SCARAjacP3DOF(Q, Qp, L)

theta1 = Q(1);
theta2 = Q(2);
theta3 = Q(3);
theta1_dot = Qp(1);
theta2_dot = Qp(2);
theta3_dot = Qp(3);

l1 = L(1);
l2 = L(2);
l3 = L(3);

theta12 = theta1 + theta2;
theta123 = theta12 + theta3;
theta12_dot = theta1_dot + theta2_dot;
theta123_dot = theta12_dot + theta3_dot;

Jp = zeros(4, 4);

% Time derivatives of partial derivatives of X
Jp(1, 1) = -l1 * cos(theta1) * theta1_dot - l2 * cos(theta12) * theta12_dot - l3 * cos(theta123) * theta123_dot;
Jp(1, 2) = -l2 * cos(theta12) * theta12_dot - l3 * cos(theta123) * theta123_dot;
Jp(1, 3) = -l3 * cos(theta123) * theta123_dot;
Jp(1, 4) = 0;

% Time derivatives of partial derivatives of Y
Jp(2, 1) = -l1 * sin(theta1) * theta1_dot - l2 * sin(theta12) * theta12_dot - l3 * sin(theta123) * theta123_dot;
Jp(2, 2) = -l2 * sin(theta12) * theta12_dot - l3 * sin(theta123) * theta123_dot;
Jp(2, 3) = -l3 * sin(theta123) * theta123_dot;
Jp(2, 4) = 0;

% Time derivatives of partial derivatives of Z
Jp(3, :) = 0; 

% Time derivatives of partial derivatives of phi
Jp(4, :) = 0; 

end
