function Jp = SCARAjacPdin4DOF(Q, Qp, L)
    % SCARAjacPdin4DOF computes the time derivative of the Jacobian matrix for a 3R SCARA robot
    l1 = L(1); l2 = L(2); l3 = L(3);
    g1 = L(4); g2 = L(5); g3 = L(6);
    Jp = zeros(8, 3);
    Jp(1,1) = -l1*cos(Q(1))*Qp(1) - l2*cos(Q(1)+Q(2))*(Qp(1)+Qp(2)) - l3*cos(Q(1)+Q(2)+Q(3))*(Qp(1)+Qp(2)+Qp(3));
    Jp(1,2) = -l2*cos(Q(1)+Q(2))*(Qp(1)+Qp(2)) - l3*cos(Q(1)+Q(2)+Q(3))*(Qp(1)+Qp(2)+Qp(3));
    Jp(1,3) = -l3*cos(Q(1)+Q(2)+Q(3))*(Qp(1)+Qp(2)+Qp(3));
    Jp(2,1) = -l1*sin(Q(1))*Qp(1) - l2*sin(Q(1)+Q(2))*(Qp(1)+Qp(2)) - l3*sin(Q(1)+Q(2)+Q(3))*(Qp(1)+Qp(2)+Qp(3));
    Jp(2,2) = -l2*sin(Q(1)+Q(2))*(Qp(1)+Qp(2)) - l3*sin(Q(1)+Q(2)+Q(3))*(Qp(1)+Qp(2)+Qp(3));
    Jp(2,3) = -l3*sin(Q(1)+Q(2)+Q(3))*(Qp(1)+Qp(2)+Qp(3));
    Jp(3,1) = -l1*cos(Q(1))*Qp(1) - g2*cos(Q(1)+Q(2))*(Qp(1)+Qp(2)) - g3*cos(Q(1)+Q(2)+Q(3))*(Qp(1)+Qp(2)+Qp(3));
    Jp(3,2) = -g2*cos(Q(1)+Q(2))*(Qp(1)+Qp(2)) - g3*cos(Q(1)+Q(2)+Q(3))*(Qp(1)+Qp(2)+Qp(3));
    Jp(3,3) = -g3*cos(Q(1)+Q(2)+Q(3))*(Qp(1)+Qp(2)+Qp(3));
    Jp(4,1) = -l1*sin(Q(1))*Qp(1) - g2*sin(Q(1)+Q(2))*(Qp(1)+Qp(2)) - g3*sin(Q(1)+Q(2)+Q(3))*(Qp(1)+Qp(2)+Qp(3));
    Jp(4,2) = -g2*sin(Q(1)+Q(2))*(Qp(1)+Qp(2)) - g3*sin(Q(1)+Q(2)+Q(3))*(Qp(1)+Qp(2)+Qp(3));
    Jp(4,3) = -g3*sin(Q(1)+Q(2)+Q(3))*(Qp(1)+Qp(2)+Qp(3));
    Jp(5,1) = 0; Jp(5,2) = 0; Jp(5,3) = 0;
    Jp(6,1) = -Qp(1)*g1*cos(Q(1)); Jp(6,2) = 0; Jp(6,3) = 0;
    Jp(7,1) = -Qp(1)*g1*sin(Q(1)); Jp(7,2) = 0; Jp(7,3) = 0;
    Jp(8,1) = 0; Jp(8,2) = 0; Jp(8,3) = 0;
end
