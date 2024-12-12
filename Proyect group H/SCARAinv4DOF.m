function Q = SCARAinv4DOF(S, L, sol, Q_prev)

    if nargin < 4
        % If Q_prev not provided, default to zeros
        Q_prev = [0; 0; 0; 0];
    end

    x = S(1);
    y = S(2);
    z = S(3);
    phi = S(4);

    l1 = L(1);
    l2 = L(2);
    l3 = L(3);

  
    x_prime = x - l3 * cos(phi);
    y_prime = y - l3 * sin(phi);

    cos_theta2 = (x_prime^2 + y_prime^2 - l1^2 - l2^2) / (2 * l1 * l2);

    if abs(cos_theta2) > 1
        error('The position is unreachable due to geometric constraints.');
    end

    if sol > 0
        theta2 = acos(cos_theta2);
    else
        theta2 = -acos(cos_theta2);
    end

    k1 = l1 + l2 * cos(theta2);
    k2 = l2 * sin(theta2);

    theta1 = atan2(y_prime, x_prime) - atan2(k2, k1);
    theta3 = phi - (theta1 + theta2);
    d4 = z; 

    Q = [theta1; theta2; theta3; d4];

    for i = 1:3
        Q(i) = Q(i) - 2*pi*round((Q(i)-Q_prev(i))/(2*pi));
    end

end
