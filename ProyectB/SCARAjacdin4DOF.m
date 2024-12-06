function J = SCARAjacdin4DOF(Q, L)
    % Extract joint variables
    theta1 = Q(1);
    theta2 = Q(2);
    theta3 = Q(3);
    d4     = Q(4);
    
    % Extract link lengths and center of mass distances
    l1 = L(1);
    l2 = L(2);
    l3 = L(3);
    g1 = L(4);
    g2 = L(5);
    g3 = L(6);
    
    % Cumulative angles
    theta12 = theta1 + theta2;
    theta123 = theta12 + theta3;
    
    % Initialize Jacobian matrix
    J = zeros(16, 4);
    
    % Rows 1-3: End-effector position
    J(1,1) = -l1*sin(theta1) - l2*sin(theta12) - l3*sin(theta123);
    J(1,2) = -l2*sin(theta12) - l3*sin(theta123);
    J(1,3) = -l3*sin(theta123);
    J(1,4) = 0;
    
    J(2,1) = l1*cos(theta1) + l2*cos(theta12) + l3*cos(theta123);
    J(2,2) = l2*cos(theta12) + l3*cos(theta123);
    J(2,3) = l3*cos(theta123);
    J(2,4) = 0;
    
    J(3,1) = 0;
    J(3,2) = 0;
    J(3,3) = 0;
    J(3,4) = 1;
    
    % Rows 4-6: Center of mass of link 3
    J(4,1) = -l1*sin(theta1) - l2*sin(theta12) - g3*sin(theta123);
    J(4,2) = -l2*sin(theta12) - g3*sin(theta123);
    J(4,3) = -g3*sin(theta123);
    J(4,4) = 0;
    
    J(5,1) = l1*cos(theta1) + l2*cos(theta12) + g3*cos(theta123);
    J(5,2) = l2*cos(theta12) + g3*cos(theta123);
    J(5,3) = g3*cos(theta123);
    J(5,4) = 0;
    
    J(6,1) = 0;
    J(6,2) = 0;
    J(6,3) = 0;
    J(6,4) = 1;
    
    % Row 7: Total orientation at end-effector
    J(7,1) = 1;
    J(7,2) = 1;
    J(7,3) = 1;
    J(7,4) = 0;
    
    % Rows 8-10: Center of mass of link 2
    J(8,1) = -l1*sin(theta1) - g2*sin(theta12);
    J(8,2) = -g2*sin(theta12);
    J(8,3) = 0;
    J(8,4) = 0;
    
    J(9,1) = l1*cos(theta1) + g2*cos(theta12);
    J(9,2) = g2*cos(theta12);
    J(9,3) = 0;
    J(9,4) = 0;
    
    J(10,1) = 0;
    J(10,2) = 0;
    J(10,3) = 0;
    J(10,4) = 1;
    
    % Row 11: Orientation at joint 2
    J(11,1) = 1;
    J(11,2) = 1;
    J(11,3) = 0;
    J(11,4) = 0;
    
    % Rows 12-14: Center of mass of link 1
    J(12,1) = -g1*sin(theta1);
    J(12,2) = 0;
    J(12,3) = 0;
    J(12,4) = 0;
    
    J(13,1) = g1*cos(theta1);
    J(13,2) = 0;
    J(13,3) = 0;
    J(13,4) = 0;
    
    J(14,1) = 0;
    J(14,2) = 0;
    J(14,3) = 0;
    J(14,4) = 1;
    
    % Row 15: Orientation at joint 1
    J(15,1) = 1;
    J(15,2) = 0;
    J(15,3) = 0;
    J(15,4) = 0;
    
    % Row 16: Prismatic joint force dynamics (z-direction)
    J(16,1) = 0;
    J(16,2) = 0;
    J(16,3) = 0;
    J(16,4) = 1; 
end
