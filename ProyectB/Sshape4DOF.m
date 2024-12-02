function res = Sshape(t, S0, dS, t1, t2, t3)
    % Calculate average velocity V
    V = dS / (0.5 * t1 + (t2 - t1) + 0.5 * (t3 - t2));
    % Acceleration during acceleration phase
    A = V / t1;
    % Deceleration during deceleration phase
    D = V / (t3 - t2);

    if t < t1
        % Acceleration phase
        res.pos = S0 + 0.5 * A * t^2;
        res.vel = A * t;
        res.acc = A;
    elseif t < t2
        % Constant velocity phase
        res.pos = S0 + 0.5 * A * t1^2 + V * (t - t1);
        res.vel = V;
        res.acc = 0;
    elseif t <= t3
        % Deceleration phase
        td = t - t2; % Time since deceleration started
        s1 = 0.5 * A * t1^2 + V * (t2 - t1); % Distance up to t2
        res.pos = S0 + s1 + V * td - 0.5 * D * td^2;
        res.vel = V - D * td;
        res.acc = -D;
    else
        res.pos = S0 + dS;
        res.vel = 0;
        res.acc = 0;
    end
end
