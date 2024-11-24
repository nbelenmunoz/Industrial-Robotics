function S = SCARAdir4DOF(Q, L)

theta1 = Q(1);
theta2 = Q(2);    
theta3 = Q(3);
d4 = Q(4);


l1 = L(1);
l2 = L(2);
l3 = L(3);     

S = zeros(3,1);

S(1) = l1*cos(theta1) + l2*cos(theta1 + theta2) + l3*cos(theta1 + theta2 + theta3);
S(2) = l1*sin(theta1) + l2*sin(theta1 + theta2) + l3*sin(theta1 + theta2 + theta3);
S(3) = d4;

end
