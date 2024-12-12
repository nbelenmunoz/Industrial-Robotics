clc; clear; close all;

% Define link lengths
l1 = 0.35;  
l2 = 0.31;  
l3 = 0.21;  
L = [l1; l2; l3];

% Define target end-effector position and orientation
X_target = -0.650;     
Y_target = 0.450;          
Z_target = 0.00;          
phi_target = pi;    

S = [X_target; Y_target; Z_target; phi_target];

try
    Q1 = SCARAinv4DOF(S, L, 1);    % Elbow-up configuration
    Q2 = SCARAinv4DOF(S, L, -1);   % Elbow-down configuration
catch ME
    disp('Inverse kinematics computation failed:');
    disp(ME.message);
    return;
end

% Plot the workspace and robot configurations
figure(1);
PlotAreaSCARA4DOF(L, 1);           
hold on;
PlotScara4DOF(Q1, L, 'r', 1);      % Plot first solution in red
PlotScara4DOF(Q2, L, 'b', 1);      % Plot second solution in blue
hold off;
title('SCARA Robot Configurations');
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
legend('Origin', 'Maximum Reach Singularity', 'Minimum Reach Singularity', ...
       'Link l1', 'Link l1+l2', 'Link l1+l2+l3', ...
       'Solution Q1', 'Solution Q2', 'Location', 'bestoutside');
grid on; % Add grid for better visualization

% Compute forward kinematics for both solutions
S1 = SCARAdir4DOF(Q1, L); 
S2 = SCARAdir4DOF(Q2, L);  

% Display the joint variables and end-effector positions
disp('First solution joint variables Q1 (theta1, theta2, theta3, d4):');
disp(Q1);

disp('Second solution joint variables Q2 (theta1, theta2, theta3, d4):');
disp(Q2);

disp('End-effector position from forward kinematics S1 (X, Y, Z, phi):');
disp(S1);

disp('End-effector position from forward kinematics S2 (X, Y, Z, phi):');
disp(S2);

epsilon = 1e-6;

checkSingularity(Q1, L, 'Q1', epsilon);
checkSingularity(Q2, L, 'Q2', epsilon);

%%  Function Definitions

function checkSingularity(Q, L, solutionName, epsilon)
   
    % Compute the Jacobian matrix
    J = SCARAjac4DOF(Q, L);
    
    % Compute determinant and rank of the Jacobian
    detJ = det(J);
    rankJ = rank(J);
    
    % Display Jacobian information
    fprintf('\n%s Solution Jacobian Det(J) = %.6f, Rank(J) = %d\n', solutionName, detJ, rankJ);
    
    % Check for singularity based on determinant and rank
    if abs(detJ) < epsilon || rankJ < 4
        warning('%s solution is in or near a singular configuration.', solutionName);
    else
        disp([solutionName ' solution is not singular.']);
    end
end
