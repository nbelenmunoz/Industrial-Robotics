function [tv_global,trajectory_joint,trajectory_joint_vel,trajectory_joint_acc]= LineParabola(Q,V_max,A,D,res)
% inputs:
% res----> time resolution you like (dt)
% A --------> max acceleration
% D-----------> max decceleration
% Q-----------> matrix of points in the jointspace(Q=[Qi,Q1,Q2,Q3],Qi=[Qj1;Qj2;...])

T_ = V_max * (1 / A + 1 / D);
dQ_ = 0.5 * V_max^2 * (1 / A + 1 / D);

[nj, nm] = size(Q);
Tmin = zeros(1, nm-1);
dq = diff(Q, 1, 2);  

for ii = 1:nm-1
    T = zeros(nj, 1);
    for j = 1:nj
        dq_abs = abs(dq(j, ii));
        if dq_abs < dQ_
            T(j) = (sqrt(A/D) + sqrt(D/A)) * sqrt(2 * dq_abs / (A + D));
        elseif dq_abs >= dQ_
            T(j) = dq_abs / V_max + V_max * (A + D) / (2 * A * D);
        else
            T(j) = T_;
        end
    end
    Tmin(ii) = max(T);
end


tmin = zeros(nj, nm-1);
tj = zeros(nj, nm);
qp = zeros(nj, nm+1);

for ii = 1:nm-1
    if ii == 1
        tj(:, ii) = Tmin(ii) - sqrt(Tmin(ii)^2 - 2 * abs(dq(:, ii)) / A);
        tmin(:, ii) = Tmin(ii) - 0.5 * tj(:, ii);
        qp(:, ii+1) = dq(:, ii) ./ tmin(:, ii);
    elseif ii == nm-1
        tj(:, ii+1) = Tmin(ii) - sqrt(Tmin(ii)^2 - 2 * abs(dq(:, ii)) / A);
        tmin(:, ii) = Tmin(ii) - 0.5 * tj(:, end);
        qp(:, ii+1) = dq(:, end) ./ tmin(:, ii);
        tj(:, ii) = abs((dq(:, ii) ./ tmin(:, ii) - dq(:, ii-1) ./ Tmin(:, ii-1)) / A);
    else
        tmin(:, ii) = Tmin(ii);
        tj(:, ii) = abs((dq(:, ii) ./ tmin(:, ii) - dq(:, ii-1) ./ tmin(:, ii-1)) / A);
        qp(:, ii+1) = dq(:, ii) ./ tmin(:, ii);
    end
end


tvex = zeros(nj, 2*nm);
tvexj = zeros(nj, nm);
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


ap = zeros(nj, nm);
bp = zeros(nj, nm);
cp = zeros(nj, nm);
for jj = 1:nm
    ap(:, jj) = (al(:, jj+1) - al(:, jj)) ./ (2 * (tvex(:, 2*jj) - tvex(:, 2*jj-1)));
    bp(:, jj) = qp(:, jj+1) - 2 * ap(:, jj) .* tvex(:, 2*jj);

    if jj == nm
        x = Q(:, end);
    else
        x = al(:, jj+1) .* tvex(:, 2*jj) + bl(:, jj+1);
    end
    cp(:, jj) = x - ap(:, jj) .* tvex(:, 2*jj).^2 - bp(:, jj) .* tvex(:, 2*jj);
end


dt = res; 
t_start = min(tvex(:));
t_end = max(tvex(:));
tv_global = t_start:dt:t_end;


trajectory_joint = zeros(nj, length(tv_global)); 
trajectory_joint_vel = zeros(nj, length(tv_global)); 
trajectory_joint_acc = zeros(nj, length(tv_global)); 


for kk=1:nj
for k = 1:length(tv_global)
    t = tv_global(k); 

    
    for jj = 1:size(tvex, 2)/2
        if t >= tvex(kk, 2*jj-1) && t <= tvex(kk, 2*jj)
            trajectory_joint(kk,k) = ap(kk, jj)*t^2 + bp(kk, jj)*t + cp(kk, jj);
            trajectory_joint_vel(kk,k) = 2*ap(kk, jj)*t + bp(kk, jj) ;
            trajectory_joint_acc(kk,k) = 2*ap(kk, jj);

            break;
        elseif jj > 1 && t >= tvex(kk, 2*jj-2) && t < tvex(kk, 2*jj-1)
            trajectory_joint(kk,k) = al(kk, jj)*t + bl(kk, jj);
            trajectory_joint_vel(kk,k) = al(kk, jj);
            trajectory_joint_acc(kk,k) = al(kk, jj)*0;

            break;
        end
    end
end
end

