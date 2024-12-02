% Define Link Lengths (meters)
l1 = 0.35;  
l2 = 0.31;  
l3 = 0.21;  

% COM Distances from Joint Axes (meters)
g1 = 0.2230181; 
g2 = 0.2313302; 
g3 = 0.0511910; 

L = [l1; l2; l3; g1; g2; g3]; 

% Masses (kg)
m1 = 4.626; 
m2 = 3.15;  
m3 = 0.433; 
m4 = 0.5;   
m = [m1; m2; m3; m4];

% Inertias about the COMs (kg·m^2)
Jg1 = 2.8534e+05 * 1e-6; 
Jg2 = 1.8614e+05 * 1e-6; 
Jg3 = 3.7115e+03 * 1e-6; 
Jg = [Jg1; Jg2; Jg3];

% Gravity Constant
g_const = 9.81; % m/s^2

% Define Waypoints (meters and radians)
Waypoints1 = [
    -0.650,  0.450, 0.000, pi;       % Point 1: Initial
    -0.250,  0.500, 0.000, pi/2;     % Point 2: Orientation before Point 3
    -0.250,  0.650, 0.000, pi/2;     % Point 3: Slightly higher
    -0.250,  0.650, 0.010, pi/2;     % Point 4: Going up
    -0.400,  0.300, 0.010, pi;       % Point 5: Intermediate for orientation change
    -0.550,  0.000, 0.010, pi;       % Point 6: In front of Point 7
    -0.720,  0.000, 0.010, pi;       % Point 7: Slightly in front
    -0.720,  0.000, 0.000, pi;       % Point 8: Going down
    -0.550,  0.000, 0.010, pi        % Point 9: Similar to Point 6
];

Waypoints2 = [
    -0.650,  0.450, 0.000, pi;       % Point 1: Initial (same as original)
     0.000,  0.500, 0.000, pi/2;     % Point 2: Shifted right to X=0
     0.000,  0.650, 0.000, pi/2;     % Point 3: Shifted right to X=0
     0.000,  0.650, 0.020, pi/2;     % Point 4: Shifted up Z=0.020
    -0.400,  0.300, 0.020, pi;       % Point 5: Shifted up Z=0.020
    -0.550,  0.000, 0.020, pi;       % Point 6: Shifted up Z=0.020
    -0.720,  0.000, 0.020, pi;       % Point 7: Shifted up Z=0.020
    -0.720,  0.000, 0.010, pi;       % Point 8: Descend to Z=0.010 (not all the way down)
    -0.550,  0.000, 0.010, pi        % Point 9: Similar to Point 6
];

Waypoints3 = [
    -0.650,  0.450, 0.000, pi;       % Point 1: Initial
     0.250,  0.500, 0.000, pi/2;     % Point 2: Shifted right to X=0.250
     0.250,  0.650, 0.000, pi/2;     % Point 3: Shifted right to X=0.250
     0.250,  0.650, 0.010, pi/2;     % Point 4: Shifted up Z=0.010
    -0.400,  0.300, 0.030, pi;       % Point 5: Shifted up Z=0.030
    -0.550,  0.000, 0.030, pi;       % Point 6: Shifted up Z=0.030
    -0.720,  0.000, 0.020, pi;       % Point 7: Shifted up Z=0.020
    -0.720,  0.000, 0.020, pi;       % Point 8: Descend to Z=0.020
    -0.550,  0.000, 0.000, pi        % Point 9: Similar to Point 6
];

Waypoints_combined = [
    Waypoints1;
    Waypoints2(2:end, :);  % Skip Point 1 to avoid duplication
    Waypoints3(2:end, :)   % Skip Point 1 to avoid duplication
];

Waypoints = Waypoints_combined;

num_waypoints = size(Waypoints, 1);
num_segments = num_waypoints - 1;

% Define Desired Speed (meters per second)
desired_speed = 0.01; 

steps_per_segment = zeros(num_segments, 1);
for seg = 1:num_segments
    distance = norm(Waypoints(seg,1:3) - Waypoints(seg+1,1:3));
    steps_per_segment(seg) = ceil((distance / desired_speed) / 0.1);
end
num_steps_total = sum(steps_per_segment);

% Preallocate Storage Variables
px_total = zeros(1, num_steps_total);
py_total = zeros(1, num_steps_total);
pz_total = zeros(1, num_steps_total);
vx_total = zeros(1, num_steps_total);
vy_total = zeros(1, num_steps_total);
vz_total = zeros(1, num_steps_total);
ax_total = zeros(1, num_steps_total);
ay_total = zeros(1, num_steps_total);
az_total = zeros(1, num_steps_total);
phi_total = zeros(1, num_steps_total);
phi_dot_total = zeros(1, num_steps_total);
phi_ddot_total = zeros(1, num_steps_total);

q1_total = zeros(1, num_steps_total);
q1p_total = zeros(1, num_steps_total);
q1pp_total = zeros(1, num_steps_total);
q2_total = zeros(1, num_steps_total);
q2p_total = zeros(1, num_steps_total);
q2pp_total = zeros(1, num_steps_total);
q3_total = zeros(1, num_steps_total);
q3p_total = zeros(1, num_steps_total);
q3pp_total = zeros(1, num_steps_total);
q4_total = zeros(1, num_steps_total);
q4p_total = zeros(1, num_steps_total);
q4pp_total = zeros(1, num_steps_total);
tau_total = zeros(4, num_steps_total);  % Joint torques
trajectory_total = zeros(num_steps_total, 3);  % End-effector positions

time_total = zeros(1, num_steps_total);

current_step = 1; % Initialize Step Counter

% Initialize Visualization
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

% Plot Workspace Area
PlotAreaSCARA4DOF(L, 1);

% Initialize Plots
h_links = plot3([0, 0], [0, 0], [0, 0], 'k-', 'LineWidth', 2);
h_joints = plot3(0, 0, 0, 'ko', 'MarkerSize', 6, 'MarkerFaceColor', 'k');
h_ee = plot3(0, 0, 0, 'ro', 'MarkerSize', 6, 'MarkerFaceColor', 'r');
h_traj = plot3(0, 0, 0, 'b-', 'LineWidth', 1.5);

max_reach = l1 + l2 + l3;
app = (max_reach + max(abs(Waypoints(:,3)))) * 1.1;
xlim([-app, app]);
ylim([-app, app]);
zlim([0, app]);

% Loop Through Each Segment
for seg = 1:num_segments
    % Define Start and End Points for This Segment
    S_start = Waypoints(seg, :)';
    S_end = Waypoints(seg + 1, :)';
    
    % Compute Deltas
    dSx = S_end(1) - S_start(1);
    dSy = S_end(2) - S_start(2);
    dSz = S_end(3) - S_start(3);
    dPhi = S_end(4) - S_start(4);
    
    tx1 = 3; tx2 = 6; tx3 = 9;      
    ty1 = 1; ty2 = 8; ty3 = 9;     
    tz1 = 2; tz2 = 7; tz3 = 9;  

    num_steps = steps_per_segment(seg);
    time_steps = linspace(0, (num_steps-1)*0.1, num_steps); % 0.1s timestep
    
    for i = 1:num_steps
        t = (i-1)*0.1; % Current Time Within the Segment
        
        resx = Sshape4DOF(t, S_start(1), dSx, tx1, tx2, tx3);
        resy = Sshape4DOF(t, S_start(2), dSy, ty1, ty2, ty3);
        resz = Sshape4DOF(t, S_start(3), dSz, tz1, tz2, tz3);
        
        % Orientation: Linear Interpolation
        phi_t = S_start(4) + dPhi * (t / ((num_steps-1)*0.1)); 
        phi_dot = dPhi / ((num_steps-1)*0.1);
        phi_ddot = 0;
        
        % Assemble State Vectors
        S = [resx.pos; resy.pos; resz.pos; phi_t];
        Sp_partial = [resx.vel; resy.vel; resz.vel; phi_dot];
        Spp_partial = [resx.acc; resy.acc; resz.acc; phi_ddot];
    
        % Inverse Kinematics
        Q1 = SCARAinv4DOF(S, L, 1);
        
        % Updated Jacobian and Dynamics
        J_full = SCARAjacdin4DOF(Q1, L);  % Obtain the full 15x4 Jacobian
        
        % Construct the full Sp vector (15x1)
        Sp_full = zeros(15, 1);
        Sp_full(1:4) = Sp_partial;  % Assign desired end-effector velocities

        % Define Weights (optional)
        weights = ones(15, 1);
        weights(1:4) = 10; % Prioritize end-effector velocities
        W = diag(weights);
        
        % Solve for Q1p using weighted least-squares
        Q1p = pinv(J_full' * W * J_full) * J_full' * W * Sp_full; % (4x15)*(15x4)*(4x1) -> (4x1)
        
        % Compute Jp (Derivative of Jacobian)
        Jp_full = SCARAjacPdin4DOF(Q1, Q1p, L);  % 15x4
        
        Q1pp = zeros(4,1); 
        
        M = computeInertiaMatrix(Q1, L, m, Jg);
        C = computeCoriolisMatrix(Q1, Q1p, L, m, Jg);
        G = computeGravityVector(Q1, L, m, g_const);
        tau = M * Q1pp + C * Q1p + G;

        % Compute Link and Joint Points
        [link_pts, joint_pts] = computeSCARAPoints(L, Q1);
       
        set(h_links, 'XData', link_pts(:,1), 'YData', link_pts(:,2), 'ZData', link_pts(:,3));
        set(h_joints, 'XData', joint_pts(:,1), 'YData', joint_pts(:,2), 'ZData', joint_pts(:,3));
        set(h_ee, 'XData', link_pts(end,1), 'YData', link_pts(end,2), 'ZData', link_pts(end,3));
    
        trajectory_total(current_step, :) = [link_pts(end,1), link_pts(end,2), link_pts(end,3)];
        set(h_traj, 'XData', trajectory_total(1:current_step,1), ...
                   'YData', trajectory_total(1:current_step,2), ...
                   'ZData', trajectory_total(1:current_step,3));
        
        drawnow;
        
        % Store Dynamics Data
        px_total(current_step) = resx.pos;   
        vx_total(current_step) = resx.vel;   
        ax_total(current_step) = resx.acc;
        
        py_total(current_step) = resy.pos;   
        vy_total(current_step) = resy.vel;   
        ay_total(current_step) = resy.acc;
        
        pz_total(current_step) = resz.pos;   
        vz_total(current_step) = resz.vel;   
        az_total(current_step) = resz.acc;
        
        phi_total(current_step) = phi_t;     
        phi_dot_total(current_step) = phi_dot;   
        phi_ddot_total(current_step) = phi_ddot;
        
        q1_total(current_step) = Q1(1); 
        q1p_total(current_step) = Q1p(1); 
        q1pp_total(current_step) = Q1pp(1);
        
        q2_total(current_step) = Q1(2); 
        q2p_total(current_step) = Q1p(2); 
        q2pp_total(current_step) = Q1pp(2);
        
        q3_total(current_step) = Q1(3); 
        q3p_total(current_step) = Q1p(3); 
        q3pp_total(current_step) = Q1pp(3);
        
        q4_total(current_step) = Q1(4); 
        q4p_total(current_step) = Q1p(4); 
        q4pp_total(current_step) = Q1pp(4);
        
        tau_total(:, current_step) = tau;
        
        % Update Time
        time_total(current_step) = (current_step-1) * 0.1; 
        
        current_step = current_step + 1;
    end
end

% Trim Unused Preallocated Space
px_total = px_total(1:current_step-1);
py_total = py_total(1:current_step-1);
pz_total = pz_total(1:current_step-1);
vx_total = vx_total(1:current_step-1);
vy_total = vy_total(1:current_step-1);
vz_total = vz_total(1:current_step-1);
ax_total = ax_total(1:current_step-1);
ay_total = ay_total(1:current_step-1);
az_total = az_total(1:current_step-1);
phi_total = phi_total(1:current_step-1);
phi_dot_total = phi_dot_total(1:current_step-1);
phi_ddot_total = phi_ddot_total(1:current_step-1);

q1_total = q1_total(1:current_step-1);
q1p_total = q1p_total(1:current_step-1);
q1pp_total = q1pp_total(1:current_step-1);
q2_total = q2_total(1:current_step-1);
q2p_total = q2p_total(1:current_step-1);
q2pp_total = q2pp_total(1:current_step-1);
q3_total = q3_total(1:current_step-1);
q3p_total = q3p_total(1:current_step-1);
q3pp_total = q3pp_total(1:current_step-1);
q4_total = q4_total(1:current_step-1);
q4p_total = q4p_total(1:current_step-1);
q4pp_total = q4pp_total(1:current_step-1);
tau_total = tau_total(:,1:current_step-1);
trajectory_total = trajectory_total(1:current_step-1, :);
time_total = 0:0.1:(current_step-2)*0.1;

% Visualization: Subplots for End-Effector Dynamics
figure;
subplot(4, 2, 1);
plot(time_total, px_total, 'b-', time_total, vx_total, 'r--', time_total, ax_total, 'g-.');
grid on; xlabel('Time (s)'); ylabel('X Dynamics (m, m/s, m/s²)');
legend({'Position', 'Velocity', 'Acceleration'}, 'Location', 'best'); title('X Axis Dynamics');

subplot(4, 2, 2);
plot(time_total, py_total, 'b-', time_total, vy_total, 'r--', time_total, ay_total, 'g-.');
grid on; xlabel('Time (s)'); ylabel('Y Dynamics (m, m/s, m/s²)');
legend({'Position', 'Velocity', 'Acceleration'}, 'Location', 'best'); title('Y Axis Dynamics');

subplot(4, 2, 3);
plot(time_total, pz_total, 'b-', time_total, vz_total, 'r--', time_total, az_total, 'g-.');
grid on; xlabel('Time (s)'); ylabel('Z Dynamics (m, m/s, m/s²)');
legend({'Position', 'Velocity', 'Acceleration'}, 'Location', 'best'); title('Z Axis Dynamics');

subplot(4, 2, 4);
plot(time_total, phi_total, 'b-', time_total, phi_dot_total, 'r--', time_total, phi_ddot_total, 'g-.');
grid on; xlabel('Time (s)'); ylabel('Phi Dynamics (rad, rad/s, rad/s²)');
legend({'Position', 'Velocity', 'Acceleration'}, 'Location', 'best'); title('Phi Dynamics');

% Subplots for Joint Dynamics
subplot(4, 2, 5);
plot(time_total, q1_total, 'b-', time_total, q1p_total, 'r--', time_total, q1pp_total, 'g-.');
grid on; xlabel('Time (s)'); ylabel('Joint 1 (rad, rad/s, rad/s²)');
legend({'Position', 'Velocity', 'Acceleration'}, 'Location', 'best'); title('Joint 1 Dynamics');

subplot(4, 2, 6);
plot(time_total, q2_total, 'b-', time_total, q2p_total, 'r--', time_total, q2pp_total, 'g-.');
grid on; xlabel('Time (s)'); ylabel('Joint 2 (rad, rad/s, rad/s²)');
legend({'Position', 'Velocity', 'Acceleration'}, 'Location', 'best'); title('Joint 2 Dynamics');

subplot(4, 2, 7);
plot(time_total, q3_total, 'b-', time_total, q3p_total, 'r--', time_total, q3pp_total, 'g-.');
grid on; xlabel('Time (s)'); ylabel('Joint 3 (rad, rad/s, rad/s²)');
legend({'Position', 'Velocity', 'Acceleration'}, 'Location', 'best'); title('Joint 3 Dynamics');

subplot(4, 2, 8);
plot(time_total, q4_total, 'b-', time_total, q4p_total, 'r--', time_total, q4pp_total, 'g-.');
grid on; xlabel('Time (s)'); ylabel('Joint 4 (m, m/s, m/s²)');
legend({'Position', 'Velocity', 'Acceleration'}, 'Location', 'best'); title('Joint 4 Dynamics');

% End-Effector Trajectory in XY Plane
figure;
plot(px_total, py_total, 'b-', 'LineWidth', 2);
grid on; xlabel('X Position (m)'); ylabel('Y Position (m)');
title('End-Effector Trajectory in XY Plane');

% Plot Joint Torques
figure;
subplot(4,1,1);
plot(time_total, tau_total(1,:), 'b-');
grid on; xlabel('Time (s)'); ylabel('Torque 1 (N·m)');
title('Joint 1 Torque');

subplot(4,1,2);
plot(time_total, tau_total(2,:), 'r-');
grid on; xlabel('Time (s)'); ylabel('Torque 2 (N·m)');
title('Joint 2 Torque');

subplot(4,1,3);
plot(time_total, tau_total(3,:), 'g-');
grid on; xlabel('Time (s)'); ylabel('Torque 3 (N·m)');
title('Joint 3 Torque');

subplot(4,1,4);
plot(time_total, tau_total(4,:), 'k-');
grid on; xlabel('Time (s)'); ylabel('Force on Joint 4 (N)');
title('Joint 4 Force');

%% Compute SCARA link and joint points
function [link_pts, joint_pts] = computeSCARAPoints(L, Q)
    % Extract joint variables
    theta1 = Q(1);
    theta2 = Q(2);
    theta3 = Q(3);
    d4     = Q(4);  

    l1 = L(1);
    l2 = L(2);
    l3 = L(3);

    theta12 = theta1 + theta2;
    theta123 = theta12 + theta3;

    p0 = [0, 0, d4]; % Base position
    p1 = p0 + [l1*cos(theta1), l1*sin(theta1), 0];
    p2 = p1 + [l2*cos(theta12), l2*sin(theta12), 0];
    p3 = p2 + [l3*cos(theta123), l3*sin(theta123), 0];

    link_pts = [p0; p1; p2; p3];
    joint_pts = [p0; p1; p2; p3];
end

% Plot SCARA workspace area
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
