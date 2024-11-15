function PlotScara4DOF(Q, L, colore, fig)

figure(fig);
hold on;

theta1 = Q(1);
theta2 = Q(2);
d3     = Q(3); 
theta4 = Q(4);

l1 = L(1);
l2 = L(2);
l3 = L(3);

x0 = 0;
y0 = 0;

x1 = l1 * cos(theta1);
y1 = l1 * sin(theta1);

x2 = x1 + l2 * cos(theta1 + theta2);
y2 = y1 + l2 * sin(theta1 + theta2);

x3 = x2 + l3 * cos(theta1 + theta2 + theta4);
y3 = y2 + l3 * sin(theta1 + theta2 + theta4);

% Plot the robot links
plot([x0, x1, x2, x3], [y0, y1, y2, y3], 'LineWidth', 2, 'Color', colore);

plot(x0, y0, 'ko', 'MarkerSize', 6, 'MarkerFaceColor', 'k'); % Base
plot(x1, y1, 'ko', 'MarkerSize', 6, 'MarkerFaceColor', 'k'); % Joint 1
plot(x2, y2, 'ko', 'MarkerSize', 6, 'MarkerFaceColor', 'k'); % Joint 2
plot(x3, y3, 'ro', 'MarkerSize', 6, 'MarkerFaceColor', 'r'); % End-effector


axis equal;
app = (l1 + l2 + l3) * 1.1;
xlim([-app, app]);
ylim([-app, app]);
xlabel('X-axis');
ylabel('Y-axis');
title('SCARA Robot Configuration with 4 DOF');

hold off;
end
