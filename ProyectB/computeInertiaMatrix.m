function M = computeInertiaMatrix(q, L, m, Jg)

    theta1 = q(1); theta2 = q(2); theta3 = q(3);
    
    l1 = L(1); l2 = L(2); l3 = L(3);
    g1 = L(4); g2 = L(5); g3 = L(6);
    
    m1 = m(1); m2 = m(2); m3 = m(3);
    
    Jg1 = Jg(1); Jg2 = Jg(2); Jg3 = Jg(3);

    c2 = cos(theta2); c23 = cos(theta2 + theta3);
    s2 = sin(theta2); s23 = sin(theta2 + theta3);
    
    M11 = Jg1 + Jg2 + Jg3 + m2*(g2^2 + 2*l1*g2*c2 + l1^2) + ...
          m3*(g3^2 + l1^2 + l2^2 + 2*l1*l2*c2 + 2*l1*g3*c23 + 2*l2*g3*cos(theta3));
    M12 = Jg2 + Jg3 + m2*(g2^2 + l1*g2*c2) + ...
          m3*(g3^2 + l1*l2*c2 + l1*g3*c23 + l2^2 + 2*l2*g3*cos(theta3));
    M13 = Jg3 + m3*(g3^2 + l1*g3*c23 + l2*g3*cos(theta3));
    M14 = 0;
    
    M22 = Jg2 + Jg3 + m2*g2^2 + m3*(g3^2 + l2^2 + 2*l2*g3*cos(theta3));
    M23 = Jg3 + m3*(g3^2 + l2*g3*cos(theta3));
    M24 = 0;
    
    M33 = Jg3 + m3*g3^2;
    M34 = 0;
    
    M44 = m(4); % Mass moving along Z-axis (prismatic joint)
    
    % Assemble the inertia matrix
    M = [M11, M12, M13, M14;
         M12, M22, M23, M24;
         M13, M23, M33, M34;
         M14, M24, M34, M44];
end
