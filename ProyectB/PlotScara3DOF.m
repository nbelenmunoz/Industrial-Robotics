function PlotScara3DOF(Q, L, colore, fig)

figure(fig)
hold on; 

q1 = Q(1);
q2 = Q(2);
q3 = Q(3);

l1 = L(1);
l2 = L(2);
l3 = L(3);

x0 = 0;
y0 = 0;

x1 = x0 + l1 * cos(q1);
y1 = y0 + l1 * sin(q1);

x2 = x1 + l2 * cos(q1 + q2);
y2 = y1 + l2 * sin(q1 + q2);

% Position of the end-effector 
x3 = x2 + l3 * cos(q1 + q2 + q3);
y3 = y2 + l3 * sin(q1 + q2 + q3);

% Plot the robot links
plot([x0, x1, x2, x3], [y0, y1, y2, y3], 'LineWidth', 2, 'Color', colore);

% Plot the joints
plot([x0, x1, x2], [y0, y1, y2], 'ko', 'MarkerSize', 5, 'MarkerFaceColor', 'k');
plot(x3, y3, 'ro', 'MarkerSize', 5, 'MarkerFaceColor', 'r'); % End-effector

% Set axis properties
axis equal;
grid on;
app = (l1 + l2 + l3) * 1.1;
xlim([-app, app]);
ylim([-app, app]);
xlabel('X');
ylabel('Y');
title('SCARA Robot Configuration');
hold off;
end
