close all;

% Setup the figure and axes
figure;
hold on;
axis equal;
xlim([-1000 1000]);
ylim([-1000 1000]);
xlabel('X (mm)');
ylabel('Y (mm)');
title('Workspace Representation');

% Define the concentric circles with radii 200, 310, 350, 600 mm
radii = [350, 660, 870];
center = [0, 0];

for r = radii
    rectangle('Position', [center(1) - r, center(2) - r, 2*r, 2*r], ...
              'Curvature', [1, 1], 'EdgeColor', 'b', 'LineWidth', 1);
end

% Define reference points
robotBase = [0,0];
homePositionE = [-600, 400];    % center of box
positionD = [-600, 0];
positionA = [-250, 650];
positionB = [0, 650];
positionC = [250, 650];

% Draw object base A, B, C
rectangle('Position', [positionA(1)-100, positionA(2)-100, 200, 200], 'FaceColor', 'g');
rectangle('Position', [positionB(1)-100, positionB(2)-100, 200, 200], 'FaceColor', 'g');
rectangle('Position', [positionC(1)-100, positionC(2)-100, 200, 200], 'FaceColor', 'g');

% Draw home position and position D
rectangle('Position', [homePositionE(1)-100, homePositionE(2)-50, 100, 100], 'FaceColor', [0.8 0.8 1], 'EdgeColor', 'b');
rectangle('Position', [positionD(1)-200, positionD(2)-125, 200, 250], 'FaceColor', [0.8 0.8 1], 'EdgeColor', 'b');

% Draw object base A, B, C, D
rectangle('Position', [positionA(1)-75, positionA(2)-75, 150, 150], 'FaceColor', [0.8 0.8 1], 'EdgeColor', 'k');
rectangle('Position', [positionB(1)-75, positionB(2)-75, 150, 150], 'FaceColor', [0.8 0.8 1], 'EdgeColor', 'k');
rectangle('Position', [positionC(1)-75, positionC(2)-75, 150, 150], 'FaceColor', [0.8 0.8 1], 'EdgeColor', 'k');
rectangle('Position', [positionD(1)-150, positionD(2)-75, 150, 150], 'FaceColor', [0.8 0.8 1], 'EdgeColor', 'k');

% Draw reference points
plot(robotBase(1), robotBase(2), 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k');

plot(homePositionE(1), homePositionE(2), 'ko', 'MarkerSize', 5, 'MarkerFaceColor', 'k');
plot(positionD(1), positionD(2), 'ko', 'MarkerSize', 5, 'MarkerFaceColor', 'k');
plot(positionA(1), positionA(2), 'ko', 'MarkerSize', 5, 'MarkerFaceColor', 'r');
plot(positionB(1), positionB(2), 'ko', 'MarkerSize', 5, 'MarkerFaceColor', 'r');
plot(positionC(1), positionC(2), 'ko', 'MarkerSize', 5, 'MarkerFaceColor', 'r');




% % Draw position
% rectangle('Position', [homePositionE(1), homePositionE(2), 50, 50], ...
%     'FaceColor', [0.8 0.8 1], 'EdgeColor', 'b');


% % Position A
% positionA = [-850, 50]; % Relative position to the center (x, y)
% widthA = 150;
% heightA = 150;
% rectangle('Position', [positionA(1), positionA(2), widthA, heightA], ...
%           'EdgeColor', 'k', 'FaceColor', 'r');
% 
% % Position B
% positionB = [-850 + 150, 50]; % Position B is next to A (based on the dimensions)
% widthB = 150;
% heightB = 150;
% rectangle('Position', [positionB(1), positionB(2), widthB, heightB], ...
%           'EdgeColor', 'k', 'FaceColor', 'g');


% Labels and other graphical elements can be added if necessary

hold off;
