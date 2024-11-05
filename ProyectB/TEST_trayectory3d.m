clc; clear all; close all;

L = [1, 0.8, 0.5];          % Link lengths [l1, l2, l3]
n_steps = 100;               % Number of animation steps


theta1_vals = linspace(0, pi/3, n_steps);    % Range for theta1
theta2_vals = linspace(0, -pi/4, n_steps);   % Range for theta2
d3_vals = linspace(0.2, 0.4, n_steps);       % Range for d3
theta4_vals = linspace(0, pi/2, n_steps);    % Range for theta4


fig = figure;
hold on;
axis equal;
axis([-2, 2, -2, 2, -0.5, 1.5]);  % Set fixed axis limits for consistency
grid on;
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
title('3D Simulation of 4-DOF SCARA Robot');
view(3);

PlotAreaSCARA4DOF(L, fig);

h_traj = plot3(0, 0, 0, 'b-', 'LineWidth', 1.5);  % Trajectory plot
h_links = plot3([0, 0], [0, 0], [0, 0], 'k-', 'LineWidth', 2);  % Links plot
h_joints = plot3(0, 0, 0, 'ko', 'MarkerSize', 6, 'MarkerFaceColor', 'k');  % Joints plot
trajectory = zeros(n_steps, 3);  % End-effector trajectory

% Animation loop
for i = 1:n_steps
    Q = [theta1_vals(i), theta2_vals(i), d3_vals(i), theta4_vals(i)];

    S = SCARAdir4DOF(Q, L);
    trajectory(i, :) = S';  % Store end-effector position

    [link_pts, joint_pts] = computeSCARAPoints(L, Q);  % Get link and joint points

    set(h_links, 'XData', link_pts(:,1), 'YData', link_pts(:,2), 'ZData', link_pts(:,3));
    set(h_joints, 'XData', joint_pts(:,1), 'YData', joint_pts(:,2), 'ZData', joint_pts(:,3));
    set(h_traj, 'XData', trajectory(1:i, 1), 'YData', trajectory(1:i, 2), 'ZData', trajectory(1:i, 3));
    pause(0.03);
end

hold off;
