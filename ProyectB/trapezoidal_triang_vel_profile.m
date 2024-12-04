% Practice 3
% Min Actuating Time; Trapezoidal and Triangular Velocity Profile
% Implementation

clear; clc; close all;

% motor specs
A = 3;  % acceleration [rad/s^2]
D = 3;  % deceleration [rad/s^2]
V_adjust = 4;  % velocity [rad/s]

% pose
q11 = 0;    % link 1 pose 1
q21 = 0;    % link 2 pose 1
q12 = 180;  % link 1 pose 2
q22 = 90;   % link 2 pose 2

% calculating delta for both joints
delta_q11 = q12 - q11;
delta_q21 = q22 - q21;
delta_qi = [delta_q11, delta_q21];  % in degrees
delta_qi = delta_qi * pi / 180;   % convert to radians

% time and rise limit calculations
time_limit = V_adjust * (1 / A + 1 / D);   % T-bar time limit [s]
rise_limit = 0.5 * V_adjust^2 * (1 / A + 1 / D);  % q-bar rise limit [rad]

% time step calculation for each joint
T = zeros(size(delta_qi));  % initialize time array to store values for each joint

for i = 1:length(delta_qi)
    % Calculate time for triangular (Case 1) and trapezoidal (Case 2) velocity profiles
    case2_deltaT = delta_qi(i) / V_adjust + V_adjust * (A + D) / (2 * A * D);    % trapezoidal velocity profile
    case1_deltaT = (sqrt(A / D) + sqrt(D / A)) * sqrt(2 * delta_qi(i) / (A + D));    % triangular velocity profile
    
    % Condition to select the correct motion profile
    if delta_qi(i) < rise_limit
        T(i) = case1_deltaT;
    else
        T(i) = case2_deltaT;
    end
end

% The minimum actuating time is the maximum of all joint times
T_min = max(T);

% Display results
disp('Actuating times for each joint:');
disp(T);
disp(['Minimum actuating time of the entire trajectory: ', num2str(T_min), ' s']);   % the trajectory can't be completed in less time
fprintf('\n')

% Step 5: Synchronize joints by adjusting speed
[T_min, max_index] = max(T);
V_adjust = zeros(1,length(delta_qi));
for i = 1:length(delta_qi)
        % Adjust velocity to synchronize with the longest time
        V_adjust(i) = (0.5*A*T_min)-(sqrt((0.5*A*T_min)^2-A*delta_qi(i)));
        fprintf('Adjusted velocity for joint %d to synchronize: %.3f rad/s\n', i, V_adjust(i));
end

% Calculate acceleration/deceleration times
fprintf('\n');
T_acc = zeros(1,length(delta_qi));
for i = 1:length(delta_qi)
    T_acc(i) = V_adjust(i)/A;
    fprintf('Acceleration/Deceleration time for joint %d to synchronize: %.3f s\n', i, T_acc(i));
end

% Plotting the velocity profile
figure;
hold on;
colors = ['b', 'r'];  % Colors for plotting different joints
for i = 1:length(delta_qi)
    t = linspace(0, T_min, 100);
    if i == max_index
        % Triangular velocity profile
        T_half = T_min / 2;
        v = zeros(size(t));
        v(t <= T_half) = A * t(t <= T_half);  % Acceleration phase
        v(t > T_half) = A * (T_min - t(t > T_half));  % Deceleration phase
    else
        % Trapezoidal velocity profile
        T_cruise = T_min - T_acc(i) - T_acc(i);
        v = zeros(size(t));
        v(t <= T_acc(i)) = A * t(t <= T_acc(i));  % Acceleration phase
        v(t > T_acc(i) & t <= (T_acc(i) + T_cruise)) = V_adjust(i);  % Constant velocity phase
        v(t > (T_acc(i) + T_cruise)) = V_adjust(i) - D * (t(t > (T_acc(i) + T_cruise)) - (T_acc(i) + T_cruise));  % Deceleration phase
    end
    plot(t, v, colors(i), 'DisplayName', ['Joint ', num2str(i)]);
end

xlabel('Time [s]');
ylabel('Velocity [rad/s]');
title('Velocity Profiles for Each Joint');
legend;
grid on;
hold off;





