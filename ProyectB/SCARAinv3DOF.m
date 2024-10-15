function Q = SCARAinv3DOF(S, L, sol)


X = S(1);
Y = S(2);
Z = S(3);
phi = S(4); 

l1 = L(1);
l2 = L(2);
l3 = L(3);

% Compute d4 (prismatic joint)
d4 = Z;

% Compute the position of the wrist center
Px = X - l3 * cos(phi);
Py = Y - l3 * sin(phi);

% Compute the distance from the base to the wrist center
r = sqrt(Px^2 + Py^2);

% Compute the angle beta using the Law of Cosines
cos_beta = (r^2 - l1^2 - l2^2) / (2 * l1 * l2);

% Check if the point is reachable
if abs(cos_beta) > 1
    error('The point is not reachable');
end

% Compute beta
beta = acos(cos_beta);

% Determine theta2 based on the solution parameter
if sol > 0
    theta2 = beta;
else
    theta2 = -beta;
end


alpha = atan2(Py, Px);
gamma = atan2(l2 * sin(theta2), l1 + l2 * cos(theta2));
theta1 = alpha - gamma;
theta3 = phi - theta1 - theta2;

% Assemble the joint variables
Q = [theta1; theta2; theta3; d4];
end
