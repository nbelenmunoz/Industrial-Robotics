clc; clear all; close all; 

l1 = 0.35;  
l2 = 0.31;  
l3 = 0.21; 

g1 = 0.2230181;
g2 = 0.2313302;
g3 = 0.0511910;

L = [l1; l2; l3; g1; g2; g3];

% Masses (in kg)
m1 = 4.626;
m2 = 3.15;
m3 = 0.433;
payload = 0;  % Initial payload mass
m4 = m1 + m2 + m3 + payload;

m = [m1; m2; m3; payload];

% Inertias about the COMs (in kg·m^2)
Jg1 = 2.8534e+05 * 1e-6;
Jg2 = 1.8614e+05 * 1e-6;
Jg3 = 3.7115e+03 * 1e-6;
Jg = [Jg1; Jg2; Jg3];

% Gravity constant (in m/s^2)
g_const = 9.81;

% Inertia matrix (with prismatic joint contribution)
M = diag([payload, payload, payload, m3, m3, m3, (Jg1 + Jg2 + Jg3), m2, m2, m2, (Jg1 + Jg2), m1, m1, m1, Jg1,m4]);

% External force vector
Fse = zeros(16, 1);  
Fse(3) = payload * g_const;  % Gravitational force (N) in the z-direction
        
% Maximum Joint Velocities (rad/s)
omega_max = [5.585; 6.981; 6.981; 13.963];  % [Shoulder; Elbow; Wrist; Prismatic]

% Safety Factor
safety_factor = 1.5;

% Required Joint Torques (Nm)
tau_joint_required = [0.2404; 0.2891; 6.6540; 89.6007];  % [Shoulder; Elbow; Wrist; Prismatic]

% Total Transmission Ratios for Each Joint
GR_total = [1/112.5; 1/90; 1/90; 1/45];  % [Shoulder; Elbow; Wrist; Prismatic]

% Calculate Motor Torque Requirements
tau_motor_required = tau_joint_required ./ GR_total;  % [27.045; 26.019; 598.86; 4032.0315]

% Apply Safety Factor
tau_max = tau_motor_required * safety_factor;  % [40.5675; 39.0285; 898.29; 6048.04725]


% Initialize Constraint Flags
Waypoints1 = [
    -0.650,  0.450, 0.000, pi;       % Point 1 
    -0.250,  0.500, 0.000, pi/2;     % Point 2
    -0.250,  0.650, 0.000, pi/2;     % Point 3
    -0.250,  0.650, 0.010, pi/2;     % Point 4 pick
    -0.250,  0.500, 0.010, pi/2;     % Point 5
    -0.400,  0.300, 0.010, pi;       % Point 6
    -0.550,  0.000, 0.010, pi;       % Point 7
    -0.720,  0.000, 0.010, pi;       % Point 8
    -0.720,  0.000, 0.000, pi;       % Point 9 place
    -0.550,  0.000, 0.000, pi;       % Point 10 
    -0.550,  0.000, 0.000, pi        % Point 11 
];

Waypoints2 = [
     0.000,  0.500, 0.000, pi/2;     % Point 2
     0.000,  0.650, 0.000, pi/2;     % Point 3
     0.000,  0.650, 0.020, pi/2;     % Point 4 pick
     0.000,  0.500, 0.020, pi/2;     % Point 5
    -0.200,  0.300, 0.020, pi;       % Point 6
    -0.550,  0.000, 0.020, pi;       % Point 7
    -0.720,  0.000, 0.020, pi;       % Point 8
    -0.720,  0.000, 0.010, pi;       % Point 9 place
    -0.550,  0.000, 0.010, pi;       % Point 10 
    -0.550,  0.000, 0.000, pi        % Point 11
];

Waypoints3 = [
     0.250,  0.500, 0.000, pi/2;     % Point 2
     0.250,  0.650, 0.000, pi/2;     % Point 3
     0.250,  0.650, 0.030, pi/2;     % Point 4 pick
     0.250,  0.500, 0.030, pi/2      % Point 5
     0.000,  0.300, 0.030, pi;       % Point 6
    -0.550,  0.000, 0.030, pi;       % Point 7
    -0.720,  0.000, 0.030, pi;       % Point 8
    -0.720,  0.000, 0.020, pi;       % Point 9 place
    -0.550,  0.000, 0.020, pi;       % point 10
    -0.550,  0.000, 0.010, pi        % Point 11
    -0.650,  0.450, 0.000, pi;       % Point 12: Initial
    -0.650,  0.450, 0.000, pi;       % Point 1: Initial
];

% Waypoints mix
Waypoints = [Waypoints1; Waypoints2; Waypoints3];
total_segments = size(Waypoints, 1) - 1;

% Waypoint Index, payload value
payload_changes = [
    4, 0.5;    % pick
    9, 0;      % place
    14, 0.5;   % pick
    19, 0;     % place
    24, 0.5;   % pick
    29, 0      % place
];


% Estimate total_time_steps based on waypoints and time_steps
time_steps = 0:0.1:1; % Time steps for each segment
time_steps_per_segment = length(time_steps);
total_time_steps = time_steps_per_segment * total_segments;
constraint_violated = false(total_time_steps, 4); 

% Initialize dynamic payload
current_payload = 0;               % Start with no payload
payload_vector = zeros(total_time_steps, 1); 
current_time_step = 1;             % Initialize time step counter


J_rot = [Jg1; Jg2; Jg3; 1];  

% Maximum Angular Accelerations
alpha_max = tau_max ./ J_rot;  

figure(1);
clf; 
hold on;
grid on;
axis equal;
xlabel('X-axis (m)');
ylabel('Y-axis (m)');
zlabel('Z-axis (m)');
title('SCARA Robot Trajectory in 3D');
view(45, 30);

% Plot workspace area
PlotAreaSCARA4DOF(L, 1);

trajectory = zeros(total_time_steps, 3);
Fq = zeros(total_time_steps, 4);
time_vec = zeros(1, total_time_steps);
px_total = zeros(1, total_time_steps);
py_total = zeros(1, total_time_steps);

S_total = zeros(total_time_steps, 4);
Sp_total = zeros(total_time_steps, 4);
Spp_total = zeros(total_time_steps, 4);
Q_total = zeros(total_time_steps, 4);
Qp_total = zeros(total_time_steps, 4);
Qpp_total = zeros(total_time_steps, 4);

h_links = plot3([0, 0], [0, 0], [0, 0], 'k-', 'LineWidth', 2);
h_joints = plot3(0, 0, 0, 'ko', 'MarkerSize', 6, 'MarkerFaceColor', 'k');
h_ee = plot3(0, 0, 0, 'ro', 'MarkerSize', 6, 'MarkerFaceColor', 'r');
h_traj = plot3(0, 0, 0, 'b-', 'LineWidth', 1.5);

% Set axis limits
max_reach = l1 + l2 + l3;
app = max_reach * 1.1;
xlim([-app, app]);
ylim([-app, app]);
zlim([0, app]);

idx = 1;
for wp_idx = 1:total_segments
    % Check if the next waypoint has a payload change
    if any(payload_changes(:,1) == wp_idx + 1)
    % Update payload dynamically
    new_payload = payload_changes(payload_changes(:,1) == wp_idx + 1, 2);
    current_payload = new_payload;

    % Update masses and inertia matrix
    m4 = m1 + m2 + m3 + current_payload;
    M = diag([current_payload, current_payload, current_payload,(Jg1 + Jg2 + Jg3), m3, m3, m3,(Jg1 + Jg2), m2, m2, m2, Jg1, m1, m1, m1, m4]);

    % Update gravitational force based on new payload
    Fse = zeros(16, 1);
    Fse(3) = current_payload * g_const;
    end
    
    % Define the current segment's start and end waypoints
    Si = Waypoints(wp_idx, :)';
    Sf = Waypoints(wp_idx + 1, :)';
    Ds = Sf - Si;

    % Extract individual components for trajectory planning
    Six = Si(1); dSx = Ds(1);
    Siy = Si(2); dSy = Ds(2);
    Siz = Si(3); dSz = Ds(3);
    Siphi = Si(4); dPhi = Ds(4);

    % Trajectory parameters (can be adjusted or made dynamic if needed)
    t1 = 1.0; % Acceleration time
    t2 = 2.0; % Constant velocity time
    t3 = 1.5; % Deceleration time

    for i = 1:length(time_steps)
        t = time_steps(i);
        
        % Compute S-curve trajectories for X, Y, Z, and Phi
        resx = Sshape4DOF(t, Six, dSx, t1, t2, t3);
        resy = Sshape4DOF(t, Siy, dSy, t1, t2, t3);
        resz = Sshape4DOF(t, Siz, dSz, t1, t2, t3);
        res_phi = Sshape4DOF(t, Siphi, dPhi, t1, t2, t3);

        S = [resx.pos; resy.pos; resz.pos; res_phi.pos];        
        Sp = [resx.vel; resy.vel; resz.vel; res_phi.vel]; 
        Spp = [resx.acc; resy.acc; resz.acc; res_phi.acc]; 

        % Scale velocities if they exceed max limits
        scaling_factors_vel = abs(Sp) ./ omega_max;
        scaling_factors_vel = max(scaling_factors_vel, 1);  % Ensure scaling factor >=1
        Sp = Sp ./ scaling_factors_vel;

        % Scale accelerations if they exceed max limits
        scaling_factors_acc = abs(Spp) ./ alpha_max;
        scaling_factors_acc = max(scaling_factors_acc, 1);  % Ensure scaling factor >=1
        Spp = Spp ./ scaling_factors_acc;

        % Store the constrained S, Sp, Spp
        S_total(idx, :) = S';
        Sp_total(idx, :) = Sp';
        Spp_total(idx, :) = Spp';

        % Inverse kinematics
        Q1 = SCARAinv4DOF(S, L, 1);

        % Jacobian and its derivative
        J = SCARAjac4DOF(Q1, L); 
        Q1p = J \ Sp; 
        Jp = SCARAjacP4DOF(Q1, Q1p, L);  
        Q1pp = J \ (Spp - Jp * Q1p); 

        % Dynamics calculations
        Sd = SCARAdirdin4DOF(Q1, L, Spp(3), Fse(3), [m1; m2; m3], current_payload);
        total_force_z = Sd(16);
        Jd = SCARAjacdin4DOF(Q1, L);
        Jpd = SCARAjacPdin4DOF(Q1, Q1p, L);
        Spp_dynamics = Jpd * Q1p + Jd * Q1pp; 
        Fsi = M * Spp_dynamics; 
        Fs = (Fse + Fsi);
        Fcq = (-Jd' * Fs) / 1000; 
        Fq(idx, :) = Fcq';
        Fq(idx, 4) = total_force_z;   

        % Store joint states
        Q_total(idx, :) = Q1';
        Qp_total(idx, :) = Q1p';
        Qpp_total(idx, :) = Q1pp';

        % Compute and plot robot configuration
        [link_pts, joint_pts] = computeSCARAPoints(L, Q1);

        set(h_links, 'XData', link_pts(:,1), 'YData', link_pts(:,2), 'ZData', link_pts(:,3));
        set(h_joints, 'XData', joint_pts(:,1), 'YData', joint_pts(:,2), 'ZData', joint_pts(:,3));
        set(h_ee, 'XData', link_pts(end,1), 'YData', link_pts(end,2), 'ZData', link_pts(end,3));

        trajectory(idx, :) = [link_pts(end,1), link_pts(end,2), link_pts(end,3)];
        px_total(idx) = link_pts(end,1);
        py_total(idx) = link_pts(end,2);
        set(h_traj, 'XData', trajectory(1:idx,1), 'YData', trajectory(1:idx,2), 'ZData', trajectory(1:idx,3));

        drawnow;

        % Update payload_vector for analysis
        payload_vector(idx) = current_payload;

        % Update time_vec
        time_vec(idx) = t + (wp_idx - 1);  % Adjust time based on segment
        idx = idx + 1;
    end
end


figure;
hold on;
plot(time_vec, Fq(:, 1), 'b', 'DisplayName', 'Torque 1');
plot(time_vec, Fq(:, 2), 'r', 'DisplayName', 'Torque 2');
plot(time_vec, Fq(:, 3), 'g', 'DisplayName', 'Torque 3');
plot(time_vec, Fq(:, 4), 'k', 'DisplayName', 'Force on Prismatic Joint');

%plot(time_vec, Fq(:, 4), 'DisplayName', 'Force on Prismatic Joint');

grid on;
xlabel('Time (s)');
ylabel('Joint Torques/Forces (Nm or N)');
legend('Location', 'best');
title('Joint Torques and Forces');
hold off;

figure;
plot(px_total, py_total, 'b-', 'LineWidth', 2);
grid on;
xlabel('X Position (m)');
ylabel('Y Position (m)');
title('End-Effector Trajectory in XY Plane');


task_vars = {'X', 'Y', 'Z', '\phi'};

for joint = 1:4 
    figure;
    p = S_total(:, joint);
    v = Sp_total(:, joint);
    a = Spp_total(:, joint);

    % Plot position
    subplot(3,1,1);
    plot(time_vec, p, 'b-');
    grid on;
    xlabel('Time (s)');
    ylabel('Position');
    title(['Position for ', task_vars{joint}]);

    % Plot velocity
    subplot(3,1,2);
    plot(time_vec, v, 'r-');
    grid on;
    xlabel('Time (s)');
    ylabel('Velocity');
    title(['Velocity for ', task_vars{joint}]);

    % Plot acceleration
    subplot(3,1,3);
    plot(time_vec, a, 'g-');
    grid on;
    xlabel('Time (s)');
    ylabel('Acceleration');
    title(['Acceleration for ', task_vars{joint}]);
end

task_varsq = {'Joint1', 'Joint2', 'Joint3', 'Prismatic joint'};
for joint = 1:4 
    pq = Q_total(:, joint);
    vq = Qp_total(:, joint);
    aq = Qpp_total(:, joint);
    figure;

     % Plot position
    subplot(3,1,1);
    plot(time_vec, pq, 'b-');
    grid on;
    xlabel('Time (s)');
    ylabel('Position');
    title(['Position for ', task_varsq{joint}]);

    % Plot velocity
    subplot(3,1,2);
    plot(time_vec, vq, 'r-');
    grid on;
    xlabel('Time (s)');
    ylabel('Velocity');
    title(['Velocity for ', task_varsq{joint}]);

    % Plot acceleration
    subplot(3,1,3);
    plot(time_vec, aq, 'g-');
    grid on;
    xlabel('Time (s)');
    ylabel('Acceleration');
    title(['Acceleration for ', task_varsq{joint}]);
end

% Plot Payload Over Time
figure;
plot(time_vec, payload_vector, 'm-', 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('Payload (kg)');
title('Dynamic Payload Over Time');
legend('Payload');


%% Helper functions
function [link_pts, joint_pts] = computeSCARAPoints(L, Q)
    % Extract joint values
    theta1 = Q(1); % Joint link 1
    theta2 = Q(2); % Joint link 2
    theta3 = Q(3);
    d4 = Q(4); % Prismatic joint (vertical movement)

    % Base position
    p0 = [0, 0, d4];  % Base joint position (Z axis)
    
    % Positions of each joint/link
    p1 = [L(1)*cos(theta1), L(1)*sin(theta1), d4];  % Joint 1 position
    p2 = p1 + [L(2)*cos(theta1 + theta2), L(2)*sin(theta1 + theta2), 0];  % Joint 2 position
    p3 = p2 + [L(3)*cos(theta1 + theta2 + theta3), L(3)*sin(theta1 + theta2 + theta3), 0];  % End-effector
    
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
