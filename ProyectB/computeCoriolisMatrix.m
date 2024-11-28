function C = computeCoriolisMatrix(q, q_dot, L, m, Jg)
    theta1 = q(1); theta2 = q(2); theta3 = q(3);
    theta1_dot = q_dot(1); theta2_dot = q_dot(2); theta3_dot = q_dot(3);
    
    l1 = L(1); l2 = L(2); g2 = L(5); g3 = L(6);

    m2 = m(2); m3 = m(3);
    
    s2 = sin(theta2); s3 = sin(theta3); s23 = sin(theta2 + theta3);
    
    h1 = -m2*l1*g2*s2 - m3*l1*l2*s2 - m3*l1*g3*s23;
    h2 = -m3*l2*g3*s3;
    
    C = zeros(4,4);
    
    C(1,1) = h1 * theta2_dot + (h1 + h2) * theta3_dot;
    C(1,2) = h1 * (theta1_dot + theta2_dot) + h2 * theta3_dot;
    C(1,3) = (h1 + h2) * (theta1_dot + theta2_dot + theta3_dot);
    
    C(2,1) = -h1 * theta1_dot;
    C(2,2) = h2 * theta3_dot;
    C(2,3) = h2 * (theta1_dot + theta2_dot + theta3_dot);
    
    C(3,1) = -(h1 + h2) * theta1_dot;
    C(3,2) = -h2 * (theta1_dot + theta2_dot);
    C(3,3) = 0;
    
    % C(4,:) and C(:,4) remain zeros
end
