function J = SCARAjac3DOF(Q, L)

% Extract joint variables
theta1 = Q(1);
theta2 = Q(2);
theta3 = Q(3);
% d4 = Q(4); 

% Extract link lengths
l1 = L(1);
l2 = L(2);
l3 = L(3);

% Compute cumulative angles
theta12 = theta1 + theta2;
theta123 = theta12 + theta3;

% Initialize the Jacobian matrix
J = zeros(3, 4);

% Compute partial derivatives for X
J(1, 1) = -l1 * sin(theta1) - l2 * sin(theta12) - l3 * sin(theta123);
J(1, 2) = -l2 * sin(theta12) - l3 * sin(theta123);
J(1, 3) = -l3 * sin(theta123);
J(1, 4) = 0;

% Compute partial derivatives for Y
J(2, 1) = l1 * cos(theta1) + l2 * cos(theta12) + l3 * cos(theta123);
J(2, 2) = l2 * cos(theta12) + l3 * cos(theta123);
J(2, 3) = l3 * cos(theta123);
J(2, 4) = 0;

% Compute partial derivatives for Z
J(3, 1) = 0;
J(3, 2) = 0;
J(3, 3) = 0;
J(3, 4) = 1;
end
