function S = SCARAdirdin4DOF(Q, L, dd4, external_force_z, masses, payload)
    theta1 = Q(1);
    theta2 = Q(2);
    theta3 = Q(3);
    d4     = Q(4);

    l1 = L(1);
    l2 = L(2);
    l3 = L(3);
    g1 = L(4);
    g2 = L(5);
    g3 = L(6);

    m1 = masses(1); 
    m2 = masses(2); 
    m3 = masses(3); 

    m4 = m1 + m2 + m3 + payload; 

    g = 9.81;

    
    S = zeros(16, 1); 
    % Cumulative angles
    theta12 = theta1 + theta2;
    theta123 = theta12 + theta3;

    % Compute end-effector position
    S(1) = l1 * cos(theta1) + l2 * cos(theta12) + l3 * cos(theta123);
    S(2) = l1 * sin(theta1) + l2 * sin(theta12) + l3 * sin(theta123);
    S(3) = d4;

    % Position of the center of mass of link 3
    S(4) = l1 * cos(theta1) + l2 * cos(theta12) + g3 * cos(theta123);
    S(5) = l1 * sin(theta1) + l2 * sin(theta12) + g3 * sin(theta123);
    S(6) = d4;

    % Total orientation at the end-effector (phi)
    S(7) = theta123;

    % Position of the center of mass of link 2
    S(8) = l1 * cos(theta1) + g2 * cos(theta12);
    S(9) = l1 * sin(theta1) + g2 * sin(theta12);
    S(10) = d4;

    % Orientation at joint 2
    S(11) = theta12;

    % Position of the center of mass of link 1
    S(12) = g1 * cos(theta1);
    S(13) = g1 * sin(theta1);
    S(14) = d4;

    % Orientation at joint 1
    S(15) = theta1;

    % Prismatic joint dynamics (Force in z-direction)
    inertiaz = m4 * dd4;
    gravityz = m4 * g;  
    totalz = inertiaz + gravityz + external_force_z;
    S(16) = totalz;
end
