clear all; close all; clc;
%% Robot Parameters
% Link length [m]
l1 = 0.35;  
l2 = 0.31;  
l3 = 0.21; 
% Center of mass [m]
g1 = 0.1904;   
g2 = 0.1640;    
g3 = 0.0512;  
g = [g1; g2; g3];

L = [l1; l2; l3; g1; g2; g3];

% Masses [Kg]
m1 = 4.626;
m2 = 3.15;
m3 = 0.433;
payload = 0;  % Initial payload mass
m4 = m1 + m2 + m3 + payload;
m = [m1; m2; m3; payload];

% Inertias about the COMs [kg·m^2]
Jg1 = 0.16641166281;  
Jg2 = 0.07728488799;  
Jg3 = 0.00368451264;  
Jg = [Jg1; Jg2; Jg3];

% Gravity constant [m/s^2]
g_const = 9.81;

% Inertia matrix 
M = diag([payload, payload, payload, m3, m3, m3, (Jg1 + Jg2 + Jg3), m2, m2, m2, (Jg1 + Jg2), m1, m1, m1, Jg1, m4]);

%External force vector
Fse = zeros(16, 1);  
Fse(16) = -(m4) * g_const;

%% Definition of waypoints for trayectory
Waypoints1 = [
    -0.650,  0.450, 0.001, pi;       % Point 1: home
    -0.250,  0.400, 0.001, pi/2      % Point 2: travel
    -0.250,  0.500, 0.001, pi/2;     % Point 3: front of pickup place
    -0.250,  0.500, 0.000, pi/2;     % Point 4:
    -0.250,  0.650, 0.000, pi/2;     % Point 5: 
    -0.250,  0.650, 0.001, pi/2;     % Point 6: pickup lifting
    -0.250,  0.500, 0.001, pi/2;     % Point 7: travel
    -0.400,  0.300, 0.001, pi;       % Point 8: travel
    -0.550,  0.000, 0.003, pi;       % Point 9: front of dropoff place/adjust height
    -0.720,  0.000, 0.003, pi;       % Point 10: position to place
    -0.720,  0.000, 0.002, pi;       % Point 11: place
    -0.550,  0.000, 0.002, pi;       % Point 12: retract
    -0.550,  0.000, 0.001, pi;       % Point 13: adjust height

];

Waypoints2 = [
     0.000,  0.400, 0.001, pi;       % Point 14
     0.000,  0.400, 0.001, pi/2;     % Point 15
     0.000,  0.500, 0.001, pi/2;     % Point 16
     0.000,  0.500, 0.000, pi/2;     % Point 17
     0.000,  0.650, 0.000, pi/2;     % Point 18 
     0.000,  0.650, 0.001, pi/2;     % Point 19 pickup 
     0.000,  0.500, 0.001, pi/2;     % Point 20 travel
    -0.200,  0.300, 0.001, pi;       % Point 21 travel
    -0.550,  0.000, 0.014, pi;       % Point 22 front of dropoff/adjust height
    -0.720,  0.000, 0.014, pi;       % Point 23 position to place
    -0.720,  0.000, 0.013, pi;       % Point 24 place
    -0.550,  0.000, 0.013, pi;       % Point 25: retract
    -0.550,  0.000, 0.001, pi;       % Point 26 adjust height
];

Waypoints3 = [
     -0.200,  0.450, 0.001, pi;       % Point
     0.250,  0.350, 0.001, pi;       % Point 27 travel 
     0.250,  0.300, 0.001, pi/2;     % Point 28
     0.250,  0.500, 0.001, pi/2;     % Point 29 
     0.250,  0.500, 0.000, pi/2;     % Point 30 
     0.250,  0.650, 0.000, pi/2;     % Point 31 
     0.250,  0.650, 0.001, pi/2;     % Point 32 pickup lifting
     0.250,  0.400, 0.001, pi/2      % Point 33 travel
     0.000,  0.300, 0.001, pi;       % Point 34 travel
    -0.550,  0.000, 0.025, pi;       % Point 35 front of dropoff/adjust height
    -0.720,  0.000, 0.025, pi;       % Point 36 position to place
    -0.720,  0.000, 0.024, pi;       % Point 37: place
    -0.550,  0.000, 0.024, pi;       % Point 38: retract
    -0.550,  0.000, 0.014, pi;       % Point 
    -0.550,  0.000, 0.001, pi;       % Point 39: adjust height
    -0.400,  0.300, 0.001, pi;       % Point 40: travel
    -0.650,  0.450, 0.001, pi;       % Point 41: back to home
];
% Waypoints mix
Waypoints = [Waypoints1; Waypoints2; Waypoints3];
total_segments = size(Waypoints, 1) - 1;

%% Trayectory parameters
Vel = 3;
Accel = 3;
Decel = Accel;
dt = 0.1;

% Conversion Waypoints to Q space with Inverse Kinematics
Q_waypoints = zeros(size(Waypoints));
for i = 1:size(Waypoints, 1)
    Q_waypoints(i, :) = SCARAinv4DOF(Waypoints(i, :), [l1, l2, l3], 1);
end
Q_waypoints = Q_waypoints';  

%% Generate Joint-Space Trajectory
% Cubic splines
[time_traj, trajectory_joint, trajectory_joint_vel, trajectory_joint_ac] = ...
    CubicSpline(Q_waypoints, Vel, Accel, Decel, dt);

%% Visualization Setup for trajectory 
figure;
clf;
hold on;
grid on;
xlabel('X-axis (m)');
ylabel('Y-axis (m)');
zlabel('Z-axis (m)');
title('SCARA Robot Trajectory in 3D');
view(45, 50);

lineWidthBase = 3;
markerSizeJoints = 6;
markerSizeEE = 6;
trajectoryLineWidth = 2.5;
waypointMarkerSize = 4.5;
waypointColor = 'green';
endEffectorColor = 'red';

% Plot the workspace area
PlotAreaSCARA4DOF(L(1:3), 1);  
axis([-1.0 1.0 -1.0 1.0 0 0.04]);


lineHandle = plot3([0, 0], [0, 0], [0, 0.35], 'k-', 'LineWidth', lineWidthBase);
set(lineHandle, 'Color', [0 0 0 0.5]); 

% Create plot handles for the robot's visualization
h_links = plot3([0, 0], [0, 0], [0, 0], 'k-', 'LineWidth', lineWidthBase);
h_joints = plot3(0, 0, 0, 'ko', 'MarkerSize', markerSizeJoints, 'MarkerFaceColor', 'black');
h_ee = plot3(0, 0, 0, 'o', 'MarkerSize', markerSizeEE, 'MarkerFaceColor', endEffectorColor, 'MarkerEdgeColor', endEffectorColor);
h_traj = plot3(0, 0, 0, 'b-', 'LineWidth', trajectoryLineWidth);

trajectory = zeros(length(time_traj), 3); 
idx = 1;

for t_idx = 1:length(time_traj)
    Q = trajectory_joint(:, t_idx);
    [link_pts, joint_pts] = computeSCARAPoints(L(1:3), Q);

    set(h_links, 'XData', link_pts(:,1), 'YData', link_pts(:,2), 'ZData', link_pts(:,3));
    set(h_joints, 'XData', joint_pts(:,1), 'YData', joint_pts(:,2), 'ZData', joint_pts(:,3));
    set(h_ee, 'XData', link_pts(end,1), 'YData', link_pts(end,2), 'ZData', link_pts(end,3));

    trajectory(idx, :) = [link_pts(end, 1), link_pts(end, 2), link_pts(end, 3)];
    set(h_traj, 'XData', trajectory(1:idx, 1), 'YData', trajectory(1:idx, 2), 'ZData', trajectory(1:idx, 3));
    pause(0.01);
    idx = idx + 1;
end


plot3(Waypoints(:,1), Waypoints(:,2), Waypoints(:,3), 'o', ...
    'MarkerSize', waypointMarkerSize, 'MarkerFaceColor', waypointColor, ...
    'MarkerEdgeColor', waypointColor);
hold on;
hLegendDot = plot3(NaN, NaN, NaN, 'o', 'MarkerSize', waypointMarkerSize, ...
    'MarkerFaceColor', 'green', 'MarkerEdgeColor', 'green');
legend(hLegendDot, 'Waypoints');

% Post-processing variables
Q_total   = trajectory_joint';       
Qp_total  = trajectory_joint_vel';   
Qpp_total = trajectory_joint_ac';    
time_vec  = time_traj;               

S_total = trajectory; 
time_vec = time_traj; 
dt = mean(diff(time_vec));

Sp_total = [diff(S_total) / dt; zeros(1, size(S_total,2))];  % N x 3
Spp_total = [diff(Sp_total) / dt; zeros(1, size(S_total,2))]; % N x 3

Q   = Q_total(end, :);     
Qp  = Qp_total(end, :)';   
Qpp = Qpp_total(end, :)';

num_segments = size(Waypoints, 1) - 1;
num_steps = length(time_vec);
distances = zeros(num_segments, 1);
for i = 1:num_segments
    distances(i) = norm(Waypoints(i + 1, 1:3) - Waypoints(i, 1:3)); % 3D distance (X, Y, Z)
end

total_distance = sum(distances);
segment_steps = round((distances / total_distance) * length(time_traj));
segment_end_indices = cumsum(segment_steps);
segment_end_indices(end) = length(time_traj); 

%% Forces and Torques calculation with dynamic payload
Fq = zeros(length(time_vec), 4);
for k = 1:length(time_vec)
    Q_k = Q_total(k, :);
    Qp_k = Qp_total(k, :)';
    Qpp_k = Qpp_total(k, :)';

    seg_idx = find(k <= segment_end_indices, 1, 'first'); 
    if isempty(seg_idx)
        seg_idx = total_segments; 
    end

    current_payload = getPayloadForSegment(seg_idx);
    m4 = m1 + m2 + m3 + current_payload;
    M = diag([current_payload, current_payload, current_payload, ...
              m3, m3, m3, (Jg1 + Jg2 + Jg3), m2, m2, m2, ...
              (Jg1 + Jg2), m1, m1, m1, Jg1, m4]);

    Fse = zeros(16, 1);  

    Fse(16) = -(m4) * g_const;
      

    % Compute Jacobians
    
    J = SCARAjacdin4DOF(Q_k, L);
    Jp = SCARAjacPdin4DOF(Q_k, Qp_k, L);

    Spp = Jp * Qp_k + J * Qpp_k;  
    Fs = M * Spp + Fse;            

    % Convert to joint-space torques/forces
    Fq(k, :) = (-J' * Fs)';
end

% Display torque and force values
max_Fq = zeros(1,size(Fq,2));
min_Fq = zeros(1,size(Fq,2));

ii = 1;
for ii = 1:size(Fq,2)
    max_Fq(ii) = max(Fq(:,ii));
    min_Fq(ii) = min(Fq(:,ii));
end

% Display the maximum and minimum forces and torques
disp('Maximum Forces and Torques (Nm):');
disp(max_Fq);
disp('Minimum Forces and Torques (Nm):');
disp(min_Fq);

%% Plots
% S space plots
task_varss = {'X-axis', 'Y-axis', 'Z-axis'};
for axis_i = 1:3
    pos = S_total(:, axis_i);
    vel = Sp_total(:, axis_i);
    acc = Spp_total(:, axis_i);

    figure('Name',['Task Space Plots for ', task_varss{axis_i}], 'NumberTitle','off');
    
    % Position
    subplot(3,1,1);
    plot(time_vec, pos, 'b-');
    grid on;
    xlabel('Time (s)');
    ylabel('Position (m)');
    title(['Position along ', task_varss{axis_i}]);

    % Velocity
    subplot(3,1,2);
    plot(time_vec, vel, 'r-');
    grid on;
    xlabel('Time (s)');
    ylabel('Velocity (m/s)');
    title(['Velocity along ', task_varss{axis_i}]);

    % Acceleration
    subplot(3,1,3);
    plot(time_vec, acc, 'g-');
    grid on;
    xlabel('Time (s)');
    ylabel('Acceleration (m/s^2)');
    title(['Acceleration along ', task_varss{axis_i}]);
end

% Q space plots
task_varsq = {'Joint 1', 'Joint 2', 'Joint 3', 'Prismatic Joint'};
for joint = 1:4
    pq = Q_total(:, joint);
    vq = Qp_total(:, joint);
    aq = Qpp_total(:, joint);

    figure('Name',['Joint Space Plots for ', task_varsq{joint}], 'NumberTitle','off');

    % Plot Position
    subplot(3,1,1);
    plot(time_vec, pq, 'b-');
    grid on;
    xlabel('Time (s)');
    ylabel('Position (rad or m)');
    title(['Position for ', task_varsq{joint}]);

    % Plot Velocity
    subplot(3,1,2);
    plot(time_vec, vq, 'r-');
    grid on;
    xlabel('Time (s)');
    ylabel('Velocity (rad/s or m/s)');
    title(['Velocity for ', task_varsq{joint}]);

    % Plot Acceleration
    subplot(3,1,3);
    plot(time_vec, aq, 'g-');
    grid on;
    xlabel('Time (s)');
    ylabel('Acceleration (rad/s^2 or m/s^2)');
    title(['Acceleration for ', task_varsq{joint}]);
end

% Q-space plots combined
figure;
task_varsq = {'Joint 1', 'Joint 2', 'Joint 3', 'Prismatic Joint'};
joint = 1;
for joint = 1:4
    pq = Q_total(:, joint);
    vq = Qp_total(:, joint);
    aq = Qpp_total(:, joint);

    % Position Plot
    subplot(3, 1, 1);
    hold on;
    plot(time_vec, pq, 'DisplayName', task_varsq{joint});
    
    % Velocity Plot
    subplot(3, 1, 2);
    hold on;
    plot(time_vec, vq, 'DisplayName', task_varsq{joint});
    
    % Acceleration Plot
    subplot(3, 1, 3);
    hold on;
    plot(time_vec, aq, 'DisplayName', task_varsq{joint});
end

% Customize plots
subplot(3, 1, 1);
grid on;
xlabel('Time (s)');
ylabel('Pos (rad)');
title('Position for all Joints');
legend('show');

subplot(3, 1, 2);
grid on;
xlabel('Time (s)');
ylabel('Vel (rad/s)');
title('Velocity for all Joints');
legend('show');

subplot(3, 1, 3);
grid on;
xlabel('Time (s)');
ylabel('Accel (rad/s^2)');
title('Acceleration for all Joints');
legend('show');


% Trayectory proyections
% XY projection of the end-effector trajectory with workspace visualization
figure;
hold on;
grid on;

% Trajectory Projection
plot(trajectory(:,1), trajectory(:,2), 'b-', 'LineWidth', 2); % Already in meters
xlabel('X Position (m)');
ylabel('Y Position (m)');
title('End-Effector Trajectory in XY Plane');
axis equal;

% Workspace Representation
xlim([-1.0 1.0]); % Limits in meters
ylim([-1.0 1.0]);

% Define concentric circles with radii in meters
radii = [0.35, 0.66, 0.87];
center = [0, 0];

for r = radii
    rectangle('Position', [center(1) - r, center(2) - r, 2*r, 2*r], ...
              'Curvature', [1, 1], 'EdgeColor', 'b', 'LineWidth', 1);
end

% Define reference points in meters
robotBase = [0, 0];
homePositionE = [-0.6, 0.4];    % Center of box
positionD = [-0.6, 0];
positionA = [-0.25, 0.65];
positionB = [0, 0.65];
positionC = [0.25, 0.65];

% Draw object base A, B, C
rectangle('Position', [positionA(1)-0.1, positionA(2)-0.1, 0.2, 0.2], 'FaceColor', 'g');
rectangle('Position', [positionB(1)-0.1, positionB(2)-0.1, 0.2, 0.2], 'FaceColor', 'g');
rectangle('Position', [positionC(1)-0.1, positionC(2)-0.1, 0.2, 0.2], 'FaceColor', 'g');

% Draw home position and position D
rectangle('Position', [homePositionE(1)-0.1, homePositionE(2)-0.05, 0.1, 0.1], 'FaceColor', [0.8 0.8 1], 'EdgeColor', 'b');
rectangle('Position', [positionD(1)-0.2, positionD(2)-0.125, 0.2, 0.25], 'FaceColor', [0.8 0.8 1], 'EdgeColor', 'b');

% Draw object base A, B, C, D
rectangle('Position', [positionA(1)-0.075, positionA(2)-0.075, 0.15, 0.15], 'FaceColor', [0.8 0.8 1], 'EdgeColor', 'k');
rectangle('Position', [positionB(1)-0.075, positionB(2)-0.075, 0.15, 0.15], 'FaceColor', [0.8 0.8 1], 'EdgeColor', 'k');
rectangle('Position', [positionC(1)-0.075, positionC(2)-0.075, 0.15, 0.15], 'FaceColor', [0.8 0.8 1], 'EdgeColor', 'k');
rectangle('Position', [positionD(1)-0.15, positionD(2)-0.075, 0.15, 0.15], 'FaceColor', [0.8 0.8 1], 'EdgeColor', 'k');

% Draw reference points
plot(robotBase(1), robotBase(2), 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
plot(homePositionE(1), homePositionE(2), 'ko', 'MarkerSize', 5, 'MarkerFaceColor', 'k');
plot(positionD(1), positionD(2), 'ko', 'MarkerSize', 5, 'MarkerFaceColor', 'k');
plot(positionA(1), positionA(2), 'ko', 'MarkerSize', 5, 'MarkerFaceColor', 'r');
plot(positionB(1), positionB(2), 'ko', 'MarkerSize', 5, 'MarkerFaceColor', 'r');
plot(positionC(1), positionC(2), 'ko', 'MarkerSize', 5, 'MarkerFaceColor', 'r');

hold off;


% XZ projection of the end-effector trajectory
figure;
plot(trajectory(:,1), trajectory(:,3), 'b-', 'LineWidth', 2);
grid on;
xlabel('X Position (m)');
ylabel('Z Position (m)');
title('End-Effector Trajectory in XZ Plane');

% Torques
figure;

% Torque on Revolute Joint 1
subplot(4,1,1);
plot(time_vec, Fq(:, 1), 'b');
title('Torque on Revolute Joint 1');
xlabel('Time (s)');
ylabel('Torque (Nm)');
grid on;

% Torque on Revolute Joint 2
subplot(4,1,2);
plot(time_vec, Fq(:, 2), 'r');
title('Torque on Revolute Joint 2');
xlabel('Time (s)');
ylabel('Torque (Nm)');
grid on;

% Torque on Revolute Joint 3
subplot(4,1,3);
plot(time_vec, Fq(:, 3), 'g');
title('Torque on Revolute Joint 3');
xlabel('Time (s)');
ylabel('Torque (Nm)');
grid on;

% Torque on Prismatic Joint
subplot(4,1,4);
Fq(:,4)=49.846/1000*Fq(:,4);
plot(time_vec, Fq(:, 4), 'k');
title('Torque on Prismatic Joint');
xlabel('Time (s)');
ylabel('Torque (Nm)');
grid on;



%% Helper functions
function [link_pts, joint_pts] = computeSCARAPoints(L, Q)
    % Extract joint values
    theta1 = Q(1); % Joint 1 angle
    theta2 = Q(2); % Joint 2 angle
    theta3 = Q(3); % Joint 3 angle
    d4 = Q(4);     % Prismatic joint displacement

    % Base position
    p0 = [0, 0, d4];
    
    % Positions of each joint/link
    p1 = [L(1)*cos(theta1), L(1)*sin(theta1), d4];  
    p2 = p1 + [L(2)*cos(theta1 + theta2), L(2)*sin(theta1 + theta2), 0]; 
    p3 = p2 + [L(3)*cos(theta1 + theta2 + theta3), L(3)*sin(theta1 + theta2 + theta3), 0]; 

    link_pts = [p0; p1; p2; p3];  % Points to form links
    joint_pts = [p0; p1; p2; p3]; % Joint positions (for markers)
end

function PlotAreaSCARA4DOF(L, fig)
    figure(fig);
    hold on;
    grid on;

    l1 = L(1);
    l2 = L(2);
    l3 = L(3);

    max_reach = l1 + l2 + l3;
    min_reach = abs(l1 - l2 - l3);

    nn = 200;
    theta = linspace(0, 2 * pi, nn);

    % Max reach
    x_max = max_reach * cos(theta);
    y_max = max_reach * sin(theta);
    plot3(x_max, y_max, zeros(size(x_max)), 'r--', 'LineWidth', 1.5);

    % Min reach
    x_min = min_reach * cos(theta);
    y_min = min_reach * sin(theta);
    plot3(x_min, y_min, zeros(size(x_min)), 'r--', 'LineWidth', 1.5);

    % Circles for link lengths
    x_l1 = l1 * cos(theta);
    y_l1 = l1 * sin(theta);
    plot3(x_l1, y_l1, zeros(size(x_l1)), 'g:', 'LineWidth', 1);

    x_l12 = (l1 + l2) * cos(theta);
    y_l12 = (l1 + l2) * sin(theta);
    plot3(x_l12, y_l12, zeros(size(x_l12)), 'g:', 'LineWidth', 1);

    x_l123 = (l1 + l2 + l3) * cos(theta);
    y_l123 = (l1 + l2 + l3) * sin(theta);
    plot3(x_l123, y_l123, zeros(size(x_l123)), 'g:', 'LineWidth', 1);

    app = max_reach * 1.1;
    xlim([-app, app]);
    ylim([-app, app]);
    zlim([0, app]);
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
end

function payload = getPayloadForSegment(seg_idx)
    if seg_idx >= 6 && seg_idx <= 11
        payload = 0.5;
    elseif seg_idx >= 19 && seg_idx <= 24
        payload = 0.5;
    elseif seg_idx >= 32 && seg_idx <= 37
        payload = 0.5;
    else
        payload = 0;
    end
end