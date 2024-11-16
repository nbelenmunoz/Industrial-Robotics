function J = SCARAjacdin4DOF(Q, L)

theta1 = Q(1);
theta2 = Q(2);
d3     = Q(3); 
theta3 = Q(4);

l1 = L(1);
l2 = L(2);
l3 = L(3);

theta12 = theta1 + theta2;
theta123 = theta1 + theta2 + theta3;

J = zeros(4, 4);

J(1,1) = -l1 * sin(theta1) - l2 * sin(theta12) - l3 * sin(theta123);
J(1,2) = -l2 * sin(theta12) - l3 * sin(theta123);
J(1,3) = 0;
J(1,4) = -l3 * sin(theta123);
J(2,1) = l1 * cos(theta1) + l2 * cos(theta12) + l3 * cos(theta123);
J(2,2) = l2 * cos(theta12) + l3 * cos(theta123);
J(2,3) = 0;
J(2,4) = l3 * cos(theta123);
J(3,1) = 0;
J(3,2) = 0;
J(3,3) = 1;  
J(3,4) = 0;
J(4,1) = 1;
J(4,2) = 1;
J(4,3) = 0;
J(4,4) = 1;

end
