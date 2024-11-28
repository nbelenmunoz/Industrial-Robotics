clc; clear all; close all;

l1 = 0.35;  
l2 = 0.31;  % m
l3 = 0.21;  % m
L = [l1; l2; l3];  % Link lengths

% COM distances from joint axes (meters)
g1 = 0.2230181; 
g2 = 0.2313302; 
g3 = 0.0511910; % m
gL = [g1; g2; g3];  % Centers of mass distances

m1 = 4.626; 
m2 = 3.15;  
m3 = 0.433; 
m4 = 0.5;   % kg 
m = [m1; m2; m3; m4];

% Inertias about the COMs
Jg1 = 2.8534e+05 * 1e-6; 
Jg2 = 1.8614e+05 * 1e-6; 
Jg3 = 3.7115e+03 * 1e-6; % kg·m^2
Jg = [Jg1; Jg2; Jg3];

% Gravity
g_const = 9.81; % m/s^2

Si = [-0.600; -0.010; 0.000; pi];       %[X; Y; Z; phi]
S_mid = [-0.440; 0.400; 0.05; pi];     % Intermediate point
Sf = [0.200; 0.600; 0.10; 1.57];      

Fx = 10; Fy = 10; Fz = 10; Mphi = 0;
Fs = [Fx; Fy; Fz; Mphi];        

% Time parameters for S-curve trajectory Segment 1 (Si to S_mid)
tx1_1 = 3; tx2_1 = 6; tx3_1 = 9;      
ty1_1 = 1; ty2_1 = 8; ty3_1 = 9;     
tz1_1 = 2; tz2_1 = 7; tz3_1 = 9;      

% Time parameters for S-curve trajectory Segment 2 (S_mid to Sf)
tx1_2 = 3; tx2_2 = 6; tx3_2 = 9;      
ty1_2 = 1; ty2_2 = 8; ty3_2 = 9;     
tz1_2 = 2; tz2_2 = 7; tz3_2 = 9;      

% Total time for each segment
t3_1 = max([tx3_1, ty3_1, tz3_1]);  % Total time for Segment 1
t3_2 = max([tx3_2, ty3_2, tz3_2]);  % Total time for Segment 2

% Initial positions and deltas
Six = Si(1); dSx = S_mid(1) - Si(1);
Siy = Si(2); dSy = S_mid(2) - Si(2);
Siz = Si(3); dSz = S_mid(3) - Si(3);       
dPhi = S_mid(4) - Si(4);  % Should be zero if phi is constant

time_steps_1 = 0:0.1:t3_1;
num_steps_1 = length(time_steps_1);

time_steps_2 = 0.1:0.1:t3_2; 
num_steps_2 = length(time_steps_2);

time_total = [time_steps_1, time_steps_1(end) + time_steps_2];
num_steps_total = num_steps_1 + num_steps_2;

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

% Initialize plots
h_links = plot3([0, 0], [0, 0], [0, 0], 'k-', 'LineWidth', 2);
h_joints = plot3(0, 0, 0, 'ko', 'MarkerSize', 6, 'MarkerFaceColor', 'k');
h_ee = plot3(0, 0, 0, 'ro', 'MarkerSize', 6, 'MarkerFaceColor', 'r');
h_traj = plot3(0, 0, 0, 'b-', 'LineWidth', 1.5);

max_reach = l1 + l2 + l3;
app = (max_reach + abs(Siz + dSz)) * 1.1;
xlim([-app, app]);
ylim([-app, app]);
zlim([0, app]);

for i = 1:num_steps_total
    if i <= num_steps_1
        % Segment 1: Si to S_mid
        t = time_steps_1(i);
        S_current = Si;
        S_next = S_mid;
        tx1 = tx1_1; tx2 = tx2_1; tx3 = tx3_1;
        ty1 = ty1_1; ty2 = ty2_1; ty3 = ty3_1;
        tz1 = tz1_1; tz2 = tz2_1; tz3 = tz3_1;
        
        % Compute S-curve for Segment 1
        resx = Sshape4DOF(t, S_current(1), dSx, tx1, tx2, tx3);
        resy = Sshape4DOF(t, S_current(2), dSy, ty1, ty2, ty3);
        resz = Sshape4DOF(t, S_current(3), dSz, tz1, tz2, tz3);
    
        % Orientation
        phi_t = S_current(4) + dPhi * (t / t3_1); % Assuming linear interpolation
        phi_dot = dPhi / t3_1;
        phi_ddot = 0;
    else
        % Segment 2: S_mid to Sf
        t_seg = time_steps_2(i - num_steps_1);
        S_current = S_mid;
        S_next = Sf;
        tx1 = tx1_2; tx2 = tx2_2; tx3 = tx3_2;
        ty1 = ty1_2; ty2 = ty2_2; ty3 = ty3_2;
        tz1 = tz1_2; tz2 = tz2_2; tz3 = tz3_2;
        
        % Compute S-curve for Segment 2
        resx = Sshape4DOF(t_seg, S_current(1), Sf(1) - S_current(1), tx1, tx2, tx3);
        resy = Sshape4DOF(t_seg, S_current(2), Sf(2) - S_current(2), ty1, ty2, ty3);
        resz = Sshape4DOF(t_seg, S_current(3), Sf(3) - S_current(3), tz1, tz2, tz3);
    
        % Orientation
        phi_t = S_current(4) + (Sf(4) - S_current(4)) * (t_seg / t3_2);
        phi_dot = (Sf(4) - S_current(4)) / t3_2;
        phi_ddot = 0;
    end
    
    % Assemble state vectors
    S = [resx.pos; resy.pos; resz.pos; phi_t];
    Sp = [resx.vel; resy.vel; resz.vel; phi_dot];
    Spp = [resx.acc; resy.acc; resz.acc; phi_ddot];

    Q1 = SCARAinv4DOF(S, L, 1);

    J = SCARAjac4DOF(Q1, L);
    Q1p = J \ Sp;
    Jp = SCARAjacP4DOF(Q1, Q1p, L);
    Q1pp = J \ (Spp - Jp * Q1p);

    q = Q1;
    q_dot = Q1p;
    q_ddot = Q1pp;

    M = computeInertiaMatrix(q, [L; g1; g2; g3], m, Jg);
    C = computeCoriolisMatrix(q, q_dot, [L; g1; g2; g3], m, Jg);
    G = computeGravityVector(q, [L; g1; g2; g3], m, g_const);
    
    tau = M * q_ddot + C * q_dot + G;

    [link_pts, joint_pts] = computeSCARAPoints(L, q);
    
    set(h_links, 'XData', link_pts(:,1), 'YData', link_pts(:,2), 'ZData', link_pts(:,3));
    set(h_joints, 'XData', joint_pts(:,1), 'YData', joint_pts(:,2), 'ZData', joint_pts(:,3));
    set(h_ee, 'XData', link_pts(end,1), 'YData', link_pts(end,2), 'ZData', link_pts(end,3));

    trajectory_total(i, :) = [link_pts(end,1), link_pts(end,2), link_pts(end,3)];
    set(h_traj, 'XData', trajectory_total(1:i,1), 'YData', trajectory_total(1:i,2), 'ZData', trajectory_total(1:i,3));
    
    drawnow;
    
    px_total(i) = resx.pos;   vx_total(i) = resx.vel;   ax_total(i) = resx.acc;
    py_total(i) = resy.pos;   vy_total(i) = resy.vel;   ay_total(i) = resy.acc;
    pz_total(i) = resz.pos;   vz_total(i) = resz.vel;   az_total(i) = resz.acc;
    phi_total(i) = phi_t;     phi_dot_total(i) = phi_dot;   phi_ddot_total(i) = phi_ddot;
    q1_total(i) = q(1); q1p_total(i) = q_dot(1); q1pp_total(i) = q_ddot(1);
    q2_total(i) = q(2); q2p_total(i) = q_dot(2); q2pp_total(i) = q_ddot(2);
    q3_total(i) = q(3); q3p_total(i) = q_dot(3); q3pp_total(i) = q_ddot(3);
    q4_total(i) = q(4); q4p_total(i) = q_dot(4); q4pp_total(i) = q_ddot(4);
    tau_total(:, i) = tau;
end

% Subplots for end-effector dynamics
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

% Subplots for joint dynamics
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

% End-effector trajectory in XY plane
figure;
plot(px_total, py_total, 'b-', 'LineWidth', 2);
grid on; xlabel('X Position (m)'); ylabel('Y Position (m)');
title('End-Effector Trajectory in XY Plane');

% Plot joint torques
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
