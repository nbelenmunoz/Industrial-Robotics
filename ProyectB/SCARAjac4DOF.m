function J = SCARAjac4DOF(Q, L)

theta1 = Q(1);
theta2 = Q(2);
d3     = Q(3);
theta4 = Q(4);

l1 = L(1);
l2 = L(2);
l3 = L(3);

theta12 = theta1 + theta2;
theta1234 = theta1 + theta2 + theta4;

% Initialize the Jacobian matrix
J = zeros(4, 4);

% Partial derivatives with respect to theta1
J(1,1) = -l1*sin(theta1) - l2*sin(theta12) - l3*sin(theta1234);
J(2,1) =  l1*cos(theta1) + l2*cos(theta12) + l3*cos(theta1234);
J(3,1) = 0; % Z does not depend on theta1
J(4,1) = 1; % phi = theta1 + theta2 + theta4

% Partial derivatives with respect to theta2
J(1,2) = -l2*sin(theta12) - l3*sin(theta1234);
J(2,2) =  l2*cos(theta12) + l3*cos(theta1234);
J(3,2) = 0; % Z does not depend on theta2
J(4,2) = 1; % phi = theta1 + theta2 + theta4

% Partial derivatives with respect to d3 
J(1,3) = 0; % X does not depend on d3
J(2,3) = 0; % Y does not depend on d3
J(3,3) = 1; % Z depends linearly on d3
J(4,3) = 0; % phi does not depend on d3

% Partial derivatives with respect to theta4
J(1,4) = -l3*sin(theta1234);
J(2,4) =  l3*cos(theta1234);
J(3,4) = 0; 
J(4,4) = 1; 

end
