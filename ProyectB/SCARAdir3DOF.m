function S = SCARAdir3DOF(Q, L)
% Direct kinematics: SCARA robot with additional degrees of freedom
% Q = [theta1; theta2; theta3; d4]
% L = [l1; l2; l3]

% Extract joint variables
theta1 = Q(1);
theta2 = Q(2);
theta3 = Q(3);
d4 = Q(4); % Vertical displacement (prismatic joint)

l1 = L(1);
l2 = L(2);
l3 = L(3);

theta12 = theta1 + theta2;
theta123 = theta12 + theta3;

% XYZ
X = l1 * cos(theta1) + l2 * cos(theta12) + l3 * cos(theta123);
Y = l1 * sin(theta1) + l2 * sin(theta12) + l3 * sin(theta123);
Z = d4; % Vertical position from prismatic joint

% Calculate end-effector orientation (phi)
phi = theta1 + theta2 + theta3;

S = [X; Y; Z; phi];
end
