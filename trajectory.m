clear; close all; clc;

%% Read csv file
% The csv file is edited ( some rows are deleted to just keep Table header)
T = readtable('gp18_kp2.csv');
tx = T.TX;
ty = T.TY;


%% reference line data（from Plot_Template2.xlsx）
ref_x = [0,3887.1]';
ref_y = [0,47]';

%% Graph
figure;
hold on;
box on;

% draw trajectory
plot(tx, ty, 'b-', 'LineWidth', 1.2, 'DisplayName', 'GP18 Trajectory');

% draw reference line (0.5m away the wall)
plot(ref_x, ref_y, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Reference Line (500 mm)');


xlabel('X (mm)');
ylabel('Y (mm)');
title('Trajectory Plot with Reference Line');
legend('Location', 'best');
axis equal;  
grid on;

% Set the range of the coordinate axes 
xlim([0, 4500]);
ylim([-600, 300]);

hold off;