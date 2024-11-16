function S = SCARAdirdin4DOF(Q, L)

theta1 = Q(1);
theta2 = Q(2);
d3     = Q(3);
theta3 = Q(4);

l1 = L(1);
l2 = L(2);
l3 = L(3);

theta12 = theta1 + theta2;
theta123 = theta1 + theta2 + theta3;

x_p = l1 * cos(theta1) + l2 * cos(theta12) + l3 * cos(theta123);
y_p = l1 * sin(theta1) + l2 * sin(theta12) + l3 * sin(theta123);
z_p = d3; 
theta = theta123;

S = [x_p; y_p; z_p; theta];

end
