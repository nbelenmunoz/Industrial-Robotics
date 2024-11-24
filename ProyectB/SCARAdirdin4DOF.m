function S = SCARAdirdin4DOF(Q, L)

    if length(Q) < 4
        error('Q must have at least 4 elements: [Q1, Q2, Q3, Q4]');
    end
    if length(L) < 6
        error('L must have at least 6 elements: [l1, l2, l3, g1, g2, g3]');
    end

    % Extract link lengths and offsets
    l1 = L(1); l2 = L(2); l3 = L(3);
    g1 = L(4); g2 = L(5); g3 = L(6);

    % Initialize output vector
    S = zeros(15, 1); 

    % Compute positions
    S(1) = l1 * cos(Q(1)) + l2 * cos(Q(1) + Q(2)) + l3 * cos(Q(1) + Q(2) + Q(3));
    S(2) = l1 * sin(Q(1)) + l2 * sin(Q(1) + Q(2)) + l3 * sin(Q(1) + Q(2) + Q(3));
    S(3) = Q(4);
    S(4) = l1 * cos(Q(1)) + l2 * cos(Q(1) + Q(2)) + g3 * cos(Q(1) + Q(2) + Q(3));
    S(5) = l1 * sin(Q(1)) + l2 * sin(Q(1) + Q(2)) + g3 * sin(Q(1) + Q(2) + Q(3));
    S(6) = Q(4);
    S(7) = Q(1)+Q(2)+Q(3);
    S(8) = l1 * cos(Q(1)) + g2 * cos(Q(1) + Q(2));
    S(9) = l1 * sin(Q(1)) + g2 * sin(Q(1) + Q(2)); 
    S(10) = Q(4);
    S(11) = Q(1) + Q(2)  ; % Total orientation
    S(12) = g1 * cos(Q(1));
    S(13) = g1 * sin(Q(1));
    S(14) = Q(4);
    S(15) = Q(1);
end