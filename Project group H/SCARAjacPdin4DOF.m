function Jp = SCARAjacPdin4DOF(Q, Qp, L)

    % Extract link lengths and center of mass distances
    l1 = L(1); l2 = L(2); l3 = L(3);
    g1 = L(4); g2 = L(5); g3 = L(6);

    % Extract joint variables and velocities
    theta1 = Q(1); theta2 = Q(2); theta3 = Q(3);
    theta1p = Qp(1); theta2p = Qp(2); theta3p = Qp(3);

    % Compute cumulative angles and their derivatives
    theta12 = theta1 + theta2;
    theta123 = theta12 + theta3;
    theta12p = theta1p + theta2p;
    theta123p = theta12p + theta3p;

    % Initialize Jp
    Jp = zeros(16, 4);

    % Rows 1-2: Time derivatives for end-effector position
    % Jp(1,1)
    Jp(1,1) = -l1*cos(theta1)*theta1p - l2*cos(theta12)*theta12p - l3*cos(theta123)*theta123p;
    Jp(1,2) = -l2*cos(theta12)*theta12p - l3*cos(theta123)*theta123p;
    Jp(1,3) = -l3*cos(theta123)*theta123p;
    Jp(1,4) = 0;

    Jp(2,1) = -l1*sin(theta1)*theta1p - l2*sin(theta12)*theta12p - l3*sin(theta123)*theta123p;
    Jp(2,2) = -l2*sin(theta12)*theta12p - l3*sin(theta123)*theta123p;
    Jp(2,3) = -l3*sin(theta123)*theta123p;
    Jp(2,4) = 0;

    % Row 3: Time derivative for z-coordinate (zero since z depends only on d4, which has zero partial derivatives in J)
    Jp(3,1) = 0;
    Jp(3,2) = 0;
    Jp(3,3) = 0;
    Jp(3,4) = 0;

    % Rows 4-5: Time derivatives for center of mass of link 3
    Jp(4,1) = -l1*cos(theta1)*theta1p - l2*cos(theta12)*theta12p - g3*cos(theta123)*theta123p;
    Jp(4,2) = -l2*cos(theta12)*theta12p - g3*cos(theta123)*theta123p;
    Jp(4,3) = -g3*cos(theta123)*theta123p;
    Jp(4,4) = 0;

    Jp(5,1) = -l1*sin(theta1)*theta1p - l2*sin(theta12)*theta12p - g3*sin(theta123)*theta123p;
    Jp(5,2) = -l2*sin(theta12)*theta12p - g3*sin(theta123)*theta123p;
    Jp(5,3) = -g3*sin(theta123)*theta123p;
    Jp(5,4) = 0;

    % Row 6: Time derivative for z-coordinate of link 3 center of mass
    Jp(6,1) = 0;
    Jp(6,2) = 0;
    Jp(6,3) = 0;
    Jp(6,4) = 0;

    % Row 7: Time derivative for orientation at end-effector (zero since partial derivatives are constants)
    Jp(7,1) = 0;
    Jp(7,2) = 0;
    Jp(7,3) = 0;
    Jp(7,4) = 0;

    % Rows 8-9: Time derivatives for center of mass of link 2
    Jp(8,1) = -l1*cos(theta1)*theta1p - g2*cos(theta12)*theta12p;
    Jp(8,2) = -g2*cos(theta12)*theta12p;
    Jp(8,3) = 0;
    Jp(8,4) = 0;

    Jp(9,1) = -l1*sin(theta1)*theta1p - g2*sin(theta12)*theta12p;
    Jp(9,2) = -g2*sin(theta12)*theta12p;
    Jp(9,3) = 0;
    Jp(9,4) = 0;

    % Row 10: Time derivative for z-coordinate of link 2 center of mass
    Jp(10,1) = 0;
    Jp(10,2) = 0;
    Jp(10,3) = 0;
    Jp(10,4) = 0;

    % Row 11: Time derivative for orientation at joint 2 (zero since partial derivatives are constants)
    Jp(11,1) = 0;
    Jp(11,2) = 0;
    Jp(11,3) = 0;
    Jp(11,4) = 0;

    % Rows 12-13: Time derivatives for center of mass of link 1
    Jp(12,1) = -g1*cos(theta1)*theta1p;
    Jp(12,2) = 0;
    Jp(12,3) = 0;
    Jp(12,4) = 0;

    Jp(13,1) = -g1*sin(theta1)*theta1p;
    Jp(13,2) = 0;
    Jp(13,3) = 0;
    Jp(13,4) = 0;

    % Row 14: Time derivative for z-coordinate of link 1 center of mass
    Jp(14,1) = 0;
    Jp(14,2) = 0;
    Jp(14,3) = 0;
    Jp(14,4) = 0;

    % Row 15: Time derivative for orientation at joint 1 (zero since partial derivatives are constants)
    Jp(15,1) = 0;
    Jp(15,2) = 0;
    Jp(15,3) = 0;
    Jp(15,4) = 0;

    Jp(16,1) = 0;
    Jp(16,2) = 0;
    Jp(16,3) = 0;
    Jp(16,4) = 0;
end
