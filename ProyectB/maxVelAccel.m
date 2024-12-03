% Maximum Motor Parameters
max_motor_omega_rpm = 6000; % Maximum angular velocity of the motor (rpm)
motor_reduction_ratio = 1/45; % Motor reduction ratio (verify appropriateness later)

% Transmission Ratios for Joints
transmission_ratios = 1 ./ [1, 2.5, 2, 2]; % [prismatic joint, shoulder, elbow, wrist]

% Maximum Joint Angular Velocities (rpm)
max_joint_omega_rpm = max_motor_omega_rpm * motor_reduction_ratio * transmission_ratios;

% Link Lengths (meters)
l1 = 0.35; % Shoulder link length
l2 = 0.31;    % Elbow link length
l3 = 0.21;    % Wrist link length
l0 = 0.40;     % Base height

link_lengths = [l0, l1, l2, l3];

% Maximum Joint Linear Velocities (m/s)
max_joint_omega_rad_s = max_joint_omega_rpm * 2 * pi / 60; % Convert from rpm to rad/s
max_joint_linear_velocity = max_joint_omega_rad_s .* link_lengths;

% Display Results for Maximum Linear Velocities
fprintf('Maximum Linear Velocity on Each Joint (m/s):\n');
disp(max_joint_linear_velocity);

% Maximum Acceleration/Deceleration Parameters
Jg0 = 0.001;          % Base inertia (kg·m^2) - placeholder, re-check this value
Jg1 = 2.8534e+05 * 1e-6; % Shoulder inertia (kg·m^2)
Jg2 = 1.8614e+05 * 1e-6;    % Elbow inertia (kg·m^2)
Jg3 = 3.7115e+03 * 1e-6;    % Wrist inertia (kg·m^2)

joint_inertias = [Jg0, Jg1, Jg2, Jg3];

% Maximum Motor Torque
max_motor_torque = 0.56; % Maximum motor torque (N·m)

% Maximum Joint Angular Accelerations (rad/s^2)
max_joint_ang_accel = max_motor_torque ./ joint_inertias;

% Maximum Joint Linear Accelerations (m/s^2)
max_joint_lin_accel = max_joint_ang_accel .* link_lengths;

% Display Results for Maximum Linear Accelerations
fprintf('Maximum Linear Acceleration on Each Joint (m/s^2):\n');
disp(max_joint_lin_accel);
