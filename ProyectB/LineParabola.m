function [global_time,trajectory_joint,trajectory_vel,trajectory_acc]= LineParabola(Q,Vmax,A,D,rt)
% Inputs:
%   rt: Time resolution (dt)
%   A : Max acceleration
%   D : Max decceleration
%   Q : Waypoints in the jointspace(Q)
% Outputs:
%   global_time     : Global Time vector
%   trajectory_joint: Matrix of joint positions at each time step.
%   trajectory_vel  : Matrix of joint velocities
%   trajectory_acc  : Matrix of joint accelerations

T_v = Vmax * (1 / A + 1 / D);
dQ = 0.5 * Vmax^2 * (1 / A + 1 / D);

[nj, nm] = size(Q);
Tmin = zeros(1, nm-1);
dq = diff(Q, 1, 2);  

for ii = 1:nm-1
    T = zeros(nj, 1);
    for j = 1:nj    
        dq_abs = abs(dq(j, ii));
        if dq_abs < dQ % Acceleration-deceleration limited
            T(j) = (sqrt(A/D) + sqrt(D/A)) * sqrt(2 * dq_abs / (A + D));
        elseif dq_abs >= dQ %include a constant velocity phase
            T(j) = dq_abs / Vmax + Vmax * (A + D) / (2 * A * D);
        else    %Extraordinary case
            T(j) = T_v;
        end
    end
    Tmin(ii) = max(T);
end


tmin = zeros(nj, nm-1);
tj = zeros(nj, nm);
qp = zeros(nj, nm+1);

for ii = 1:nm-1
    if ii == 1  % First segment: Starts with acceleration; no "previous velocity."
        tj(:, ii) = Tmin(ii) - sqrt(Tmin(ii)^2 - 2 * abs(dq(:, ii)) / A);
        tmin(:, ii) = Tmin(ii) - 0.5 * tj(:, ii);
        qp(:, ii+1) = dq(:, ii) ./ tmin(:, ii);
    elseif ii == nm-1 % Last segment: Ends with deceleration; no "next velocity."
        tj(:, ii+1) = Tmin(ii) - sqrt(Tmin(ii)^2 - 2 * abs(dq(:, ii)) / A);
        tmin(:, ii) = Tmin(ii) - 0.5 * tj(:, end);
        qp(:, ii+1) = dq(:, end) ./ tmin(:, ii);
        tj(:, ii) = abs((dq(:, ii) ./ tmin(:, ii) - dq(:, ii-1) ./ Tmin(:, ii-1)) / A);
    else    % Middle segment: Need to blend the velocity from the previous segment.
        tmin(:, ii) = Tmin(ii);
        tj(:, ii) = abs((dq(:, ii) ./ tmin(:, ii) - dq(:, ii-1) ./ tmin(:, ii-1)) / A);
        qp(:, ii+1) = dq(:, ii) ./ tmin(:, ii);
    end
end

% Timing for the trayectory
tvex = zeros(nj, 2*nm); %Tracks the global timing for all phases
tvexj = zeros(nj, nm); % Tracks the midpoint times for the blending phases of each segment
for jj = 1:nm
    if jj == 1
        tvexj(:, jj+1) = tj(:, jj) / 2;
    else
        tvexj(:, jj+1) = tvexj(:, jj) + tmin(:, jj-1);
    end
    tvex(:, 2*jj) = tvex(:, 2*jj-1) + tj(:, jj);
    if jj ~= nm
        tvex(:, 2*jj+1) = tvex(:, 2*jj) + tmin(:, jj) - (tj(:, jj) / 2) - (tj(:, jj+1) / 2);
    end
end


al = qp;
bl = zeros(nj, nm+1);
for jj = 2:nm
    bl(:, jj) = Q(:, jj-1) - al(:, jj) .* tvexj(:, jj);
end

% For each segment, compute the quadratic, linear, and constant coefficients for the parabolic blending phase
ap = zeros(nj, nm); % Quadratic
bp = zeros(nj, nm); % Linear
cp = zeros(nj, nm); % Constant
for jj = 1:nm  
    ap(:, jj) = (al(:, jj+1) - al(:, jj)) ./ (2 * (tvex(:, 2*jj) - tvex(:, 2*jj-1)));
    bp(:, jj) = qp(:, jj+1) - 2 * ap(:, jj) .* tvex(:, 2*jj);

    if jj == nm % For last segment
        x = Q(:, end);
    else
        x = al(:, jj+1) .* tvex(:, 2*jj) + bl(:, jj+1);
    end
    cp(:, jj) = x - ap(:, jj) .* tvex(:, 2*jj).^2 - bp(:, jj) .* tvex(:, 2*jj);
end


rt = rt; 
t_start = min(tvex(:));
t_end = max(tvex(:));
global_time = t_start:rt:t_end;


trajectory_joint = zeros(nj, length(global_time)); 
trajectory_vel = zeros(nj, length(global_time)); 
trajectory_acc = zeros(nj, length(global_time)); 


for kk=1:nj
for k = 1:length(global_time)
    t = global_time(k); 

    
    for jj = 1:size(tvex, 2)/2
        if t >= tvex(kk, 2*jj-1) && t <= tvex(kk, 2*jj) % parabolic blending phase
            trajectory_joint(kk,k) = ap(kk, jj)*t^2 + bp(kk, jj)*t + cp(kk, jj);
            trajectory_vel(kk,k) = 2*ap(kk, jj)*t + bp(kk, jj) ;
            trajectory_acc(kk,k) = 2*ap(kk, jj);

            break;
        elseif jj > 1 && t >= tvex(kk, 2*jj-2) && t < tvex(kk, 2*jj-1) % linear phase
            trajectory_joint(kk,k) = al(kk, jj)*t + bl(kk, jj);
            trajectory_vel(kk,k) = al(kk, jj);
            trajectory_acc(kk,k) = al(kk, jj)*0;

            break;
        end
    end
end
end

