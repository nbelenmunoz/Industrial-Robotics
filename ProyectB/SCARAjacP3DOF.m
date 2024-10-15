function Jp = SCARAjacP3DOF(Q, Qp, L)

% Extract joint variables and their derivatives
theta1 = Q(1);
theta2 = Q(2);
theta3 = Q(3);
% d4 = Q(4); % Prismatic joint, not used in Jp calculation

theta1_dot = Qp(1);
theta2_dot = Qp(2);
theta3_dot = Qp(3);
% d4_dot = Qp(4); 

% Extract link lengths
l1 = L(1);
l2 = L(2);
l3 = L(3);

% Compute cumulative angles and their derivatives
theta12 = theta1 + theta2;
theta123 = theta12 + theta3;

theta12_dot = theta1_dot + theta2_dot;
theta123_dot = theta12_dot + theta3_dot;

% Initialize the time derivative of the Jacobian matrix
Jp = zeros(3, 4);

% Compute time derivatives for X components
Jp(1, 1) = -l1 * cos(theta1) * theta1_dot - l2 * cos(theta12) * theta12_dot - l3 * cos(theta123) * theta123_dot;
Jp(1, 2) = -l2 * cos(theta12) * theta12_dot - l3 * cos(theta123) * theta123_dot;
Jp(1, 3) = -l3 * cos(theta123) * theta123_dot;
Jp(1, 4) = 0;

% Compute time derivatives for Y components
Jp(2, 1) = -l1 * sin(theta1) * theta1_dot - l2 * sin(theta12) * theta12_dot - l3 * sin(theta123) * theta123_dot;
Jp(2, 2) = -l2 * sin(theta12) * theta12_dot - l3 * sin(theta123) * theta123_dot;
Jp(2, 3) = -l3 * sin(theta123) * theta123_dot;
Jp(2, 4) = 0;

% Compute time derivatives for Z components 
Jp(3, :) = 0; 
end
