function PlotAreaSCARA(L, fig)
% Plot the working area of a SCARA robot with three links
% L = [l1; l2; l3]; % Link lengths
% fig: figure number

figure(fig);
hold off;
plot([0], [0], 'ko'); % Plot the origin
hold on;
grid on;

% Extract link lengths
l1 = L(1);
l2 = L(2);
l3 = L(3);

% Compute maximum and minimum reach
max_reach = l1 + l2 + l3;
min_reach = abs(l1 - l2 - l3);

% Number of points to plot the circles
nn = 200;

% Angles for plotting circles
theta = linspace(0, 2 * pi, nn);

% Circle representing the maximum reach
x_max = max_reach * cos(theta);
y_max = max_reach * sin(theta);
plot(x_max, y_max, 'r', 'LineWidth', 2); % Outer boundary

% Circle representing the minimum reach 
x_min = min_reach * cos(theta);
y_min = min_reach * sin(theta);
plot(x_min, y_min, 'r', 'LineWidth', 2); % Inner boundary

% Circle for l1
x_l1 = l1 * cos(theta);
y_l1 = l1 * sin(theta);
plot(x_l1, y_l1);

% Circle for l1 + l2
x_l12 = (l1 + l2) * cos(theta);
y_l12 = (l1 + l2) * sin(theta);
plot(x_l12, y_l12);

% Circle for l1 + l2 + l3
x_l123 = (l1 + l2 + l3) * cos(theta);
y_l123 = (l1 + l2 + l3) * sin(theta);
plot(x_l123, y_l123);

% Set axis properties
axis equal;
app = max_reach * 1.1;
xlim([-app, app]);
ylim([-app, app]);
xlabel('X');
ylabel('Y');
title('SCARA Robot Workspace');
hold off;
end
