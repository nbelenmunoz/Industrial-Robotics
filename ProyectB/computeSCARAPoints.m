function [link_pts, joint_pts] = computeSCARAPoints(L, Q)
    % Extract joint values
    theta1 = Q(1); %joint link 1
    theta2 = Q(2); %joint link 2
    theta3 = Q(3);
    d4 = Q(4); %prismatic rotation
    

    % positions
    p0 = [0, 0, d4];  % Base joint position (Z axis)
    p1 = [L(1)*cos(theta1), L(1)*sin(theta1), d4];  % Joint 1 position
    p2 = p1 + [L(2)*cos(theta1 + theta2), L(2)*sin(theta1 + theta2), 0];  % Joint 2 position
    p3 = p2 + [L(3)*cos(theta1 + theta2 + theta3), L(3)*sin(theta1 + theta2 + theta3), 0];  % End-effector
    link_pts = [p0; p1; p2; p3];  % Points to form links
    joint_pts = [p0; p1; p2; p3]; % Joint positions (for markers)
end

