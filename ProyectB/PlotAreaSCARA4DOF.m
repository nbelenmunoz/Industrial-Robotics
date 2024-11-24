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
 % maximum reach 
     x_max = max_reach * cos(theta); % it is a circle representing the max reach of the robot
     y_max = max_reach * sin(theta);
     plot3(x_max, y_max, zeros(size(x_max)), 'r', 'LineWidth', 1.5); 
     
    % minimum reach 
     x_min = min_reach * cos(theta);
     y_min = min_reach * sin(theta);
     plot3(x_min, y_min, zeros(size(x_min)), 'r', 'LineWidth', 1.5); % Inner boundary (XY plane)
     
     % ws for l1
     x_l1 = l1 * cos(theta);
     y_l1 = l1 * sin(theta);
     plot3(x_l1, y_l1, zeros(size(x_l1)), 'g:', 'LineWidth', 1);
     
     % ws for l1 + l2
     x_l12 = (l1 + l2) * cos(theta);
     y_l12 = (l1 + l2) * sin(theta);
     plot3(x_l12, y_l12, zeros(size(x_l12)), 'g:', 'LineWidth', 1);
     
     % ws for l1 + l2 + l3
     x_l123 = (l1 + l2 + l3) * cos(theta);
     y_l123 = (l1 + l2 + l3) * sin(theta);
     plot3(x_l123, y_l123, zeros(size(x_l123)), 'g:', 'LineWidth', 1);
     
     app = max_reach * 1.1;
     xlim([-app, app]);
     ylim([-app, app]);
     
end


% for i=0:nn
%      fi=2*pi/nn*i;
%      x_l1 = l1 * cos(fi); y_l1 = l1 * sin(fi);
%      x_l12 = (l1 + l2) * cos(fi); y_l12 = (l1 + l2) * sin(fi);
%      x_max = max_reach * cos(fi); y_max = max_reach * sin(fi);
%      x_min = min_reach * cos(fi); y_min = min_reach * sin(fi);
%       if i~=0
%   plot([x_l1old x_l1],[y_l1old y_l1],'LineWidth',1,'color','green');
%   plot([x_l12old x_l12],[y_l12old y_l12],'LineWidth',1,'color','green');
%   plot([x_maxold  x_max],[y_maxold y_max],'LineWidth',2,'color','green');
%   plot([x_minold x_min],[y_minold y_min],'LineWidth',2,'color','green');
%       end 
%       x_l1old =x_l1; y_l1old =y_l1;
%       x_l12old =x_l12; y_l12old =y_l12;
%       x_maxold =x_max; y_maxold =y_max;
%       x_minold=x_min ; y_minold=y_min;
%       %x_minold = x_min; x_minold =y_min;
%      end
%  hold on
