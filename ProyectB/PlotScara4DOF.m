function PlotScara4DOF(Q, L, colore, fig)

figure(fig);
hold on;

% Extract joint variables and link lengths
theta1 = Q(1);
theta2 = Q(2);
theta3 = Q(3);
d4     = Q(4); % Not used in 2D plot


l1 = L(1);
l2 = L(2);
l3 = L(3);

% Base position
x0 = 0;
y0 = 0;

% Position of the first joint
x1 = l1 * cos(theta1);
y1 = l1 * sin(theta1);

% Position of the second joint
x2 = x1 + l2 * cos(theta1 + theta2);
y2 = y1 + l2 * sin(theta1 + theta2);

% Position of the end-effector (third joint)
x3 = x2 + l3 * cos(theta1 + theta2 + theta3);
y3 = y2 + l3 * sin(theta1 + theta2 + theta3);

% Plot the robot links
plot([x0, x1, x2, x3], [y0, y1, y2, y3], 'LineWidth', 2, 'Color', colore);

% Plot the joints
plot(x0, y0, 'ko', 'MarkerSize', 6, 'MarkerFaceColor', 'k'); % Base
plot(x1, y1, 'ko', 'MarkerSize', 6, 'MarkerFaceColor', 'k'); % Joint 1
plot(x2, y2, 'ko', 'MarkerSize', 6, 'MarkerFaceColor', 'k'); % Joint 2
plot(x3, y3, 'ro', 'MarkerSize', 6, 'MarkerFaceColor', 'r'); % End-effector

% Set axis properties
axis equal;
app = (l1 + l2 + l3) * 1.1;
xlim([-app, app]);
ylim([-app, app]);

% Labels and title
xlabel('X-axis');
ylabel('Y-axis');
title('SCARA Robot Configuration with 4 DOF');

hold off;
end
