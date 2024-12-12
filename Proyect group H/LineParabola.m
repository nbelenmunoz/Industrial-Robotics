function [total_time,trajectory_pos,trajectory_vel,trajectory_acc]= LineParabola(Q,Vmax,Accel,Decel,rt)
% Inputs:
%   rt: Time resolution (dt)
%   Vmax: Max velocity
%   Accel : Max acceleration
%   Decel : Max decceleration
%   Q : Waypoints in the jointspace(Q)
% Outputs:
%   total_time      : Global Time vector
%   trajectory_pos  : Matrix of joint positions at each time step.
%   trajectory_vel  : Matrix of joint velocities
%   trajectory_acc  : Matrix of joint accelerations

T_v = Vmax * (1 / Accel + 1 / Decel);   % limit time
dQ = 0.5 * Vmax^2 * (1 / Accel + 1 / Decel);    % limit rise

[nj, np] = size(Q); % number of joints, number of points
Tmin = zeros(1, np-1);
dq = diff(Q, 1, 2);   

for ii = 1:np-1
    T = zeros(nj, 1);
    for j = 1:nj    
        dq_abs = abs(dq(j, ii));
        if dq_abs < dQ % Acceleration-deceleration limited
            T(j) = (sqrt(Accel/Decel) + sqrt(Decel/Accel)) * sqrt(2 * dq_abs / (Accel + Decel));
        elseif dq_abs >= dQ %include a constant velocity phase
            T(j) = dq_abs / Vmax + Vmax * (Accel + Decel) / (2 * Accel * Decel);
        else    %Extraordinary case
            T(j) = T_v;
        end
    end
    Tmin(ii) = max(T);  % Assign minimum actuating time
end


tmin = zeros(nj, np-1);
tj = zeros(nj, np);
qp = zeros(nj, np+1);

for ii = 1:np-1
    if ii == 1  % First segment: Starts with acceleration; no "previous velocity."
        tj(:, ii) = Tmin(ii) - sqrt(Tmin(ii)^2 - 2 * abs(dq(:, ii)) / Accel);
        tmin(:, ii) = Tmin(ii) - 0.5 * tj(:, ii);
        qp(:, ii+1) = dq(:, ii) ./ tmin(:, ii);
    elseif ii == np-1 % Last segment: Ends with deceleration; no "next velocity."
        tj(:, ii+1) = Tmin(ii) - sqrt(Tmin(ii)^2 - 2 * abs(dq(:, ii)) / Accel);
        tmin(:, ii) = Tmin(ii) - 0.5 * tj(:, end);
        qp(:, ii+1) = dq(:, end) ./ tmin(:, ii);
        tj(:, ii) = abs((dq(:, ii) ./ tmin(:, ii) - dq(:, ii-1) ./ Tmin(:, ii-1)) / Accel);
    else    % Middle segment: Need to blend the velocity from the previous segment.
        tmin(:, ii) = Tmin(ii);
        tj(:, ii) = abs((dq(:, ii) ./ tmin(:, ii) - dq(:, ii-1) ./ tmin(:, ii-1)) / Accel);
        qp(:, ii+1) = dq(:, ii) ./ tmin(:, ii);
    end
end

% Timing for the trayectory
time_segments = zeros(nj, 2*np); % Holds the start and end times for motion segments (parabolic and linear)
time_mid = zeros(nj, np); % Tracks the midpoint times for the blending phases of each segment
for jj = 1:np
    if jj == 1
        time_mid(:, jj+1) = tj(:, jj) / 2;
    else
        time_mid(:, jj+1) = time_mid(:, jj) + tmin(:, jj-1);
    end
    time_segments(:, 2*jj) = time_segments(:, 2*jj-1) + tj(:, jj);
    if jj ~= np
        time_segments(:, 2*jj+1) = time_segments(:, 2*jj) + tmin(:, jj) - (tj(:, jj) / 2) - (tj(:, jj+1) / 2);
    end
end


al = qp;    % Line coefficient
bl = zeros(nj, np+1);   % Line coefficient
for jj = 2:np
    bl(:, jj) = Q(:, jj-1) - al(:, jj) .* time_mid(:, jj);
end

% For each segment, compute the quadratic, linear, and constant coefficients for the parabolic blending phase
ap = zeros(nj, np); % Parabola coefficient - Quadratic  
bp = zeros(nj, np); % Parabola coefficient - Linear
cp = zeros(nj, np); % Parabola coefficient - Constant
for jj = 1:np  
    ap(:, jj) = (al(:, jj+1) - al(:, jj)) ./ (2 * (time_segments(:, 2*jj) - time_segments(:, 2*jj-1)));
    bp(:, jj) = qp(:, jj+1) - 2 * ap(:, jj) .* time_segments(:, 2*jj);

    if jj == np % For last segment
        x = Q(:, end);
    else
        x = al(:, jj+1) .* time_segments(:, 2*jj) + bl(:, jj+1);
    end
    cp(:, jj) = x - ap(:, jj) .* time_segments(:, 2*jj).^2 - bp(:, jj) .* time_segments(:, 2*jj);
end

% Initialize trajectory vectors
rt = rt; 
t_start = min(time_segments(:));
t_end = max(time_segments(:));
total_time = t_start:rt:t_end;
trajectory_pos = zeros(nj, length(total_time)); 
trajectory_vel = zeros(nj, length(total_time)); 
trajectory_acc = zeros(nj, length(total_time)); 

% Compute trajectory 
for kk=1:nj
for k = 1:length(total_time)
    t = total_time(k); 
    for jj = 1:size(time_segments, 2)/2
        if t >= time_segments(kk, 2*jj-1) && t <= time_segments(kk, 2*jj) % parabolic blending phase
            trajectory_pos(kk,k) = ap(kk, jj)*t^2 + bp(kk, jj)*t + cp(kk, jj);
            trajectory_vel(kk,k) = 2*ap(kk, jj)*t + bp(kk, jj) ;
            trajectory_acc(kk,k) = 2*ap(kk, jj);

            break;
        elseif jj > 1 && t >= time_segments(kk, 2*jj-2) && t < time_segments(kk, 2*jj-1) % linear phase
            trajectory_pos(kk,k) = al(kk, jj)*t + bl(kk, jj);
            trajectory_vel(kk,k) = al(kk, jj);
            trajectory_acc(kk,k) = al(kk, jj)*0;

            break;
        end
    end
end
end