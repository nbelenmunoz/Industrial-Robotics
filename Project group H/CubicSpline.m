function [tv_global, trajectory_joint, trajectory_joint_vel, trajectory_joint_acc] = CubicSpline(Q, V_max, A, D, res)
    % Cubic Spline Trajectory Planning with Velocity and Acceleration Continuity
    % Inputs:
    % Q       ---> matrix of joint waypoints [nj x nm]
    % V_max   ---> max velocity (scalar or [nj x 1] vector)
    % A       ---> max acceleration (scalar or [nj x 1] vector)
    % D       ---> max deceleration (scalar or [nj x 1] vector)
    % res     ---> time resolution (dt)
    
    % Outputs:
    % tv_global ---> global time vector
    % trajectory_joint ---> position values at each time step
    % trajectory_joint_vel ---> velocity values at each time step
    % trajectory_joint_acc ---> acceleration values at each time step

    [nj, nm] = size(Q); % Number of joints and waypoints

    % Ensure V_max, A, and D are column vectors
    if isscalar(V_max)
        V_max = V_max * ones(nj, 1);
    elseif isvector(V_max) && length(V_max) == nj
        V_max = V_max(:);
    else
        error('V_max must be a scalar or a vector with length equal to number of joints.');
    end

    if isscalar(A)
        A = A * ones(nj, 1);
    elseif isvector(A) && length(A) == nj
        A = A(:);
    else
        error('A must be a scalar or a vector with length equal to number of joints.');
    end

    if isscalar(D)
        D = D * ones(nj, 1);
    elseif isvector(D) && length(D) == nj
        D = D(:);
    else
        error('D must be a scalar or a vector with length equal to number of joints.');
    end

    % Compute minimum time per segment based on constraints
    dq = diff(Q, 1, 2); % Joint displacements
    Tmin = zeros(1, nm-1);
    for ii = 1:nm-1
        distance = abs(dq(:, ii));
        t_acc = V_max ./ A;
        d_acc = 0.5 * A .* t_acc.^2;
        is_triangle = distance < (2 * d_acc);
        Tmin_per_joint = zeros(nj, 1);
        Tmin_per_joint(is_triangle) = 2 * sqrt(distance(is_triangle) ./ A(is_triangle));
        Tmin_per_joint(~is_triangle) = (2 * t_acc(~is_triangle)) + ((distance(~is_triangle) - 2 * d_acc(~is_triangle)) ./ V_max(~is_triangle));
        Tmin(ii) = max(Tmin_per_joint);
    end

    % Global time allocation
    t_cumulative = [0, cumsum(Tmin)];
    tv_global = 0:res:t_cumulative(end);

    % Initialize trajectories
    trajectory_joint = zeros(nj, length(tv_global));
    trajectory_joint_vel = zeros(nj, length(tv_global));
    trajectory_joint_acc = zeros(nj, length(tv_global));

    % Compute cubic spline coefficients for each joint
    coeffs = zeros(nj, nm-1, 4); % [a3, a2, a1, a0]
    for j = 1:nj
        for k = 1:nm-1
            T = Tmin(k);
            q0 = Q(j, k);
            q1 = Q(j, k+1);
            v0 = 0; % Boundary velocity at start of segment
            v1 = 0; % Boundary velocity at end of segment

            % Compute coefficients based on boundary conditions
            a3 = (2*q0 - 2*q1 + v0*T + v1*T) / T^3;
            a2 = (-3*q0 + 3*q1 - 2*v0*T - v1*T) / T^2;
            a1 = v0;
            a0 = q0;

            coeffs(j, k, :) = [a3, a2, a1, a0];
        end
    end

    % Evaluate trajectory at each global time step
    for k = 1:length(tv_global)
        t = tv_global(k);
        seg = find(t >= t_cumulative, 1, 'last');
        if seg == length(t_cumulative)
            seg = seg - 1;
        end
        T_local = t - t_cumulative(seg);
        if T_local > Tmin(seg)
            T_local = Tmin(seg); % Clamp to segment duration
        end

        for j = 1:nj
            % Retrieve coefficients for current joint and segment
            a3 = coeffs(j, seg, 1);
            a2 = coeffs(j, seg, 2);
            a1 = coeffs(j, seg, 3);
            a0 = coeffs(j, seg, 4);

            % Compute position, velocity, acceleration
            trajectory_joint(j, k) = a3*T_local^3 + a2*T_local^2 + a1*T_local + a0;
            trajectory_joint_vel(j, k) = 3*a3*T_local^2 + 2*a2*T_local + a1;
            trajectory_joint_acc(j, k) = 6*a3*T_local + 2*a2;
        end
    end
end
