function PlotScara3D(Q, L, colore, fig)

figure(fig);
clf; % Clear the figure
hold on;
grid on;

% Extract joint variables and link lengths
theta1 = Q(1);
theta2 = Q(2);
d3     = Q(3); % Prismatic joint
theta4 = Q(4);

l1 = L(1);
l2 = L(2);
l3 = L(3);

% Base position (with vertical displacement d3)
x0 = 0;
y0 = 0;
z0 = d3;

% Position of the first joint
x1 = x0 + l1 * cos(theta1);
y1 = y0 + l1 * sin(theta1);
z1 = z0;

% Position of the second joint
x2 = x1 + l2 * cos(theta1 + theta2);
y2 = y1 + l2 * sin(theta1 + theta2);
z2 = z1;

% Position of the end-effector
x3 = x2 + l3 * cos(theta1 + theta2 + theta4);
y3 = y2 + l3 * sin(theta1 + theta2 + theta4);
z3 = z2;

% Plot the robot links
plot3([x0, x1], [y0, y1], [z0, z1], 'Color', colore, 'LineWidth', 2);
plot3([x1, x2], [y1, y2], [z1, z2], 'Color', colore, 'LineWidth', 2);
plot3([x2, x3], [y2, y3], [z2, z3], 'Color', colore, 'LineWidth', 2);

% Plot the joints
plot3(x0, y0, z0, 'ko', 'MarkerSize', 6, 'MarkerFaceColor', 'k'); % Base
plot3(x1, y1, z1, 'ko', 'MarkerSize', 6, 'MarkerFaceColor', 'k'); % Joint 1
plot3(x2, y2, z2, 'ko', 'MarkerSize', 6, 'MarkerFaceColor', 'k'); % Joint 2
plot3(x3, y3, z3, 'ro', 'MarkerSize', 6, 'MarkerFaceColor', 'r'); % End-effector

% Set axis limits and labels
axis equal;
app = (l1 + l2 + l3 + abs(d3)) * 1.1;
xlim([-app, app]);
ylim([-app, app]);
zlim([0, app]);

xlabel('X-axis (mm)');
ylabel('Y-axis (mm)');
zlabel('Z-axis (mm)');
title('SCARA Robot Configuration in 3D with Prismatic Base');

% Set view angle
view(45, 30);

hold off;
end
