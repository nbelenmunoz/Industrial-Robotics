function S= SCARAdir(Q,L) %Q position and L the vector with the characteristics of the robot
% direct kinematics: SCARA robot
l1=L(1);
l2=L(2);
S= zeros(2,1);
S(1)= l1*cos(Q(1))+l2*cos(Q(1)+Q(2));
S(2)= l1*sin(Q(1))+l2*sin(Q(1)+Q(2));
end
