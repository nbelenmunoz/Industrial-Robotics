clc; clear all; close all;

l1 = 300;  
l2 = 200;  
l3 = 160;  
L = [l1; l2; l3];  

Si = [-400; -100; 150; pi];       %[X; Y; Z; phi]
Sf = [-600; -10; 100; pi];     
Ds = Sf - Si;                   

Fx = 10; Fy = 10; Fz = 10; Mphi = 10;
Fs = [Fx; Fy; Fz; Mphi];        

% Time parameters for S-curve trajectory
tx1 = 3; tx2 = 6; tx3 = 9;      
ty1 = 1; ty2 = 8; ty3 = 9;     
tz1 = 2; tz2 = 7; tz3 = 9;      % Time parameters for Z
t3 = max([tx3, ty3, tz3]);      % Total time

Six = Si(1); dSx = Ds(1);
Siy = Si(2); dSy = Ds(2);
Siz = Si(3); dSz = Ds(3);       % Initial Z position and delta Z
dPhi = Ds(4);                  

figure(1);
clf; 
hold on;
grid on;
axis equal;
xlabel('X-axis (mm)');
ylabel('Y-axis (mm)');
zlabel('Z-axis (mm)');
title('SCARA Robot Trajectory in 3D');
view(45, 30);

% Plot workspace area
PlotAreaSCARA4DOF(L, 1);

time_steps = 0:0.1:t3;
num_steps = length(time_steps);

time = zeros(1, num_steps);
px = zeros(1, num_steps);
py = zeros(1, num_steps);
pz = zeros(1, num_steps);
vx = zeros(1, num_steps);
vy = zeros(1, num_steps);
vz = zeros(1, num_steps);
ax = zeros(1, num_steps);
ay = zeros(1, num_steps);
az = zeros(1, num_steps);
phi = zeros(1, num_steps);
phi_dot_arr = zeros(1, num_steps);
phi_ddot_arr = zeros(1, num_steps);
q1 = zeros(1, num_steps);
q1p = zeros(1, num_steps);
q1pp = zeros(1, num_steps);
q2 = zeros(1, num_steps);
q2p = zeros(1, num_steps);
q2pp = zeros(1, num_steps);
q3 = zeros(1, num_steps);
q3p = zeros(1, num_steps);
q3pp = zeros(1, num_steps);
q4 = zeros(1, num_steps);
q4p = zeros(1, num_steps);
q4pp = zeros(1, num_steps);
Fq = zeros(num_steps, 4);
trajectory = zeros(num_steps, 3);  %end-effector positions

h_links = plot3([0, 0], [0, 0], [0, 0], 'k-', 'LineWidth', 2);
h_joints = plot3(0, 0, 0, 'ko', 'MarkerSize', 6, 'MarkerFaceColor', 'k');
h_ee = plot3(0, 0, 0, 'ro', 'MarkerSize', 6, 'MarkerFaceColor', 'r');
h_traj = plot3(0, 0, 0, 'b-', 'LineWidth', 1.5);

% Set axis limits
max_reach = l1 + l2 + l3;
app = (max_reach + abs(Siz + dSz)) * 1.1;
xlim([-app, app]);
ylim([-app, app]);
zlim([0, app]);

% Animation loop
for i = 1:num_steps
    t = time_steps(i);
    % Compute S-curve trajectories for X, Y, and Z
    resx = Sshape(t, Six, dSx, tx1, tx2, tx3);
    resy = Sshape(t, Siy, dSy, ty1, ty2, ty3);
    resz = Sshape(t, Siz, dSz, tz1, tz2, tz3);  % Compute Z trajectory
    
    % Orientation
    phi_t = Si(4) + (Sf(4) - Si(4)) * (t / t3); 
    phi_dot = (Sf(4) - Si(4)) / t3;             
    phi_ddot = 0;                               
    
    % Assemble state vectors
    S = [resx.pos; resy.pos; resz.pos; phi_t];        
    Sp = [resx.vel; resy.vel; resz.vel; phi_dot]; 
    Spp = [resx.acc; resy.acc; resz.acc; phi_ddot]; 
    
    % Inverse kinematics
    Q1 = SCARAinv4DOF(S, L, 1);
    
    % Jacobian and its derivative
    J = SCARAjac4DOF(Q1, L); 
    Q1p = J \ Sp; 
    Jp = SCARAjacP4DOF(Q1, Q1p, L);  
    Q1pp = J \ (Spp - Jp * Q1p); 

    % Joint torques/forces
    Fq(i, :) = -(J') * Fs;  
    
    [link_pts, joint_pts] = computeSCARAPoints(L, Q1);
    
    set(h_links, 'XData', link_pts(:,1), 'YData', link_pts(:,2), 'ZData', link_pts(:,3));
    set(h_joints, 'XData', joint_pts(:,1), 'YData', joint_pts(:,2), 'ZData', joint_pts(:,3));
    set(h_ee, 'XData', link_pts(end,1), 'YData', link_pts(end,2), 'ZData', link_pts(end,3));

    trajectory(i, :) = [link_pts(end,1), link_pts(end,2), link_pts(end,3)];
    set(h_traj, 'XData', trajectory(1:i,1), 'YData', trajectory(1:i,2), 'ZData', trajectory(1:i,3));
    
    drawnow;  

    time(i) = t;
    px(i) = resx.pos;   
    py(i) = resy.pos;   
    pz(i) = resz.pos;   
    vx(i) = resx.vel;   
    vy(i) = resy.vel;   
    vz(i) = resz.vel;   
    ax(i) = resx.acc;   
    ay(i) = resy.acc;   
    az(i) = resz.acc;   
    phi(i) = phi_t;  
    phi_dot_arr(i) = phi_dot;  
    phi_ddot_arr(i) = phi_ddot;  
    q1(i) = Q1(1); q1p(i) = Q1p(1); q1pp(i) = Q1pp(1);
    q2(i) = Q1(2); q2p(i) = Q1p(2); q2pp(i) = Q1pp(2);
    q3(i) = Q1(3); q3p(i) = Q1p(3); q3pp(i) = Q1pp(3);
    q4(i) = Q1(4); q4p(i) = Q1p(4); q4pp(i) = Q1pp(4);
end

% Subplots for dynamics
figure;
subplot(4, 2, 1);
plot(time, px, 'b-', time, vx, 'r--', time, ax, 'g-.');
grid on;
xlabel('Time (s)');
ylabel('X Dynamics');
legend({'Position', 'Velocity', 'Acceleration'});
title('X Axis Dynamics');

% Subplot 2: End-Effector Y Dynamics
subplot(4, 2, 2);
plot(time, py, 'b-', time, vy, 'r--', time, ay, 'g-.');
grid on;
xlabel('Time (s)');
ylabel('Y Dynamics');
legend({'Position', 'Velocity', 'Acceleration'});
title('Y Axis Dynamics');

% Subplot 3: End-Effector Z Dynamics
subplot(4, 2, 3);
plot(time, pz, 'b-', time, vz, 'r--', time, az, 'g-.');
grid on;
xlabel('Time (s)');
ylabel('Z Dynamics');
legend({'Position', 'Velocity', 'Acceleration'});
title('Z Axis Dynamics');

% Subplot 4: End-Effector Phi Dynamics
subplot(4, 2, 4);
plot(time, phi, 'b-', time, phi_dot_arr, 'r--', time, phi_ddot_arr, 'g-.');
grid on;
xlabel('Time (s)');
ylabel('Phi Dynamics');
legend({'Position', 'Velocity', 'Acceleration'});
title('Phi Dynamics');

% Subplot 5: Joint 1 Dynamics
subplot(4, 2, 5);
plot(time, q1, 'b-', time, q1p, 'r--', time, q1pp, 'g-.');
grid on;
xlabel('Time (s)');
ylabel('Joint 1');
legend({'Position', 'Velocity', 'Acceleration'});
title('Joint 1 Dynamics');

% Subplot 6: Joint 2 Dynamics
subplot(4, 2, 6);
plot(time, q2, 'b-', time, q2p, 'r--', time, q2pp, 'g-.');
grid on;
xlabel('Time (s)');
ylabel('Joint 2');
legend({'Position', 'Velocity', 'Acceleration'});
title('Joint 2 Dynamics');

% Subplot 7: Joint 3 Dynamics
subplot(4, 2, 7);
plot(time, q3, 'b-', time, q3p, 'r--', time, q3pp, 'g-.');
grid on;
xlabel('Time (s)');
ylabel('Joint 3');
legend({'Position', 'Velocity', 'Acceleration'});
title('Joint 3 Dynamics');

% Subplot 8: Joint 4 Dynamics
subplot(4, 2, 8);
plot(time, q4, 'b-', time, q4p, 'r--', time, q4pp, 'g-.');
grid on;
xlabel('Time (s)');
ylabel('Joint 4');
legend({'Position', 'Velocity', 'Acceleration'});
title('Joint 4 Dynamics');

% End-Effector Trajectory in XY Plane
figure;
plot(px, py, 'b-', 'LineWidth', 2);
grid on;
xlabel('X Position (mm)');
ylabel('Y Position (mm)');
title('End-Effector Trajectory in XY Plane');

% Joint Torques and Forces
figure;
plot(time, Fq(:, 1), 'b-', time, Fq(:, 2), 'r--', time, Fq(:, 3), 'g-.', time, Fq(:, 4), 'k:');
grid on;
xlabel('Time (s)');
ylabel('Joint Torques/Forces');
legend({'Torque 1', 'Torque 2', 'Torque 3', 'Force on Prismatic Joint'});
title('Joint Torques and Forces');

%% Compute SCARA Points Function
function [link_pts, joint_pts] = computeSCARAPoints(L, Q)
    % Extract joint variables
    theta1 = Q(1);
    theta2 = Q(2);
    theta3 = Q(3);
    d4     = Q(4);  % Prismatic base joint (vertical movement)

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

%% Plot SCARA Workspace Area Function
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
    xlabel('X (mm)');
    ylabel('Y (mm)');
    zlabel('Z (mm)');
end



