function Jp = SCARAjacPdin4DOF(Q, Qp, L)

    l1 = L(1); l2 = L(2); l3 = L(3);
    g1 = L(4); g2 = L(5); g3 = L(6);

    theta1 = Q(1); theta2 = Q(2); theta3 = Q(3);
    theta1p = Qp(1); theta2p = Qp(2); theta3p = Qp(3);

    theta12 = theta1 + theta2;
    theta123 = theta12 + theta3;
    theta12p = theta1p + theta2p;
    theta123p = theta12p + theta3p;

    Jp = zeros(15, 4);

    Jp(1,1) = -l1*cos(theta1)*theta1p - l2*cos(theta12)*theta12p - l3*cos(theta123)*theta123p;
    Jp(1,2) = -l2*cos(theta12)*theta12p - l3*cos(theta123)*theta123p;
    Jp(1,3) = -l3*cos(theta123)*theta123p;
    Jp(1,4) = 0;

    Jp(2,1) = -l1*sin(theta1)*theta1p - l2*sin(theta12)*theta12p - l3*sin(theta123)*theta123p;
    Jp(2,2) = -l2*sin(theta12)*theta12p - l3*sin(theta123)*theta123p;
    Jp(2,3) = -l3*sin(theta123)*theta123p;
    Jp(2,4) = 0;

    Jp(3,1) = 0;
    Jp(3,2) = 0;
    Jp(3,3) = 0;
    Jp(3,4) = 0;
    
    Jp(4,1) = -l1*cos(theta1)*theta1p - l2*cos(theta12)*theta12p - g3*cos(theta123)*theta123p;
    Jp(4,2) = -l2*cos(theta12)*theta12p - g3*cos(theta123)*theta123p;
    Jp(4,3) = -g3*cos(theta123)*theta123p;
    Jp(4,4) = 0;

    Jp(5,1) = -l1*sin(theta1)*theta1p - l2*sin(theta12)*theta12p - g3*sin(theta123)*theta123p;
    Jp(5,2) = -l2*sin(theta12)*theta12p - g3*sin(theta123)*theta123p;
    Jp(5,3) = -g3*sin(theta123)*theta123p;
    Jp(5,4) = 0;

    Jp(6,1) = 0;
    Jp(6,2) = 0;
    Jp(6,3) = 0;
    Jp(6,4) = 0;

    Jp(7,1) = 0;
    Jp(7,2) = 0;
    Jp(7,3) = 0;
    Jp(7,4) = 0;

    Jp(8,1) = -l1*cos(theta1)*theta1p - g2*cos(theta12)*theta12p;
    Jp(8,2) = -g2*cos(theta12)*theta12p;
    Jp(8,3) = 0;
    Jp(8,4) = 0;

    Jp(9,1) = -l1*sin(theta1)*theta1p - g2*sin(theta12)*theta12p;
    Jp(9,2) = -g2*sin(theta12)*theta12p;
    Jp(9,3) = 0;
    Jp(9,4) = 0;

    Jp(10,1) = 0;
    Jp(10,2) = 0;
    Jp(10,3) = 0;
    Jp(10,4) = 0;

    Jp(11,1) = 0;
    Jp(11,2) = 0;
    Jp(11,3) = 0;
    Jp(11,4) = 0;

    Jp(12,1) = -g1*cos(theta1)*theta1p;
    Jp(12,2) = 0;
    Jp(12,3) = 0;
    Jp(12,4) = 0;

    Jp(13,1) = -g1*sin(theta1)*theta1p;
    Jp(13,2) = 0;
    Jp(13,3) = 0;
    Jp(13,4) = 0;

    Jp(14,1) = 0;
    Jp(14,2) = 0;
    Jp(14,3) = 0;
    Jp(14,4) = 0;

    Jp(15,1) = 0;
    Jp(15,2) = 0;
    Jp(15,3) = 0;
    Jp(15,4) = 0;
end
