```matlab
% Clear workspace, close all figures, and clear command window.
clear; close all; clc;

%% Read CSV file containing recorded trajectory data.
% Note: The CSV file has been preprocessed—extraneous rows removed, leaving only the header and data.
T = readtable('gp18_kp2.csv');
tx = T.TX;  % Extract X coordinates of the trajectory (in mm)
ty = T.TY;  % Extract Y coordinates of the trajectory (in mm)

%% Define reference line data (manually extracted from Plot_Template2.xlsx).
% This line represents the ideal path located 500 mm (0.5 m) away from the wall.
ref_x = [0, 3887.1]';  % X coordinates of reference line (mm)
ref_y = [0, 47]';      % Y coordinates of reference line (mm)

%% Plot trajectory and reference line
figure;
hold on;
box on;

% Plot the actual robot trajectory in blue.
plot(tx, ty, 'b-', 'LineWidth', 1.2, 'DisplayName', 'GP18 Trajectory');

% Plot the desired reference path as a black dashed line.
plot(ref_x, ref_y, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Reference Line (500 mm)');

% Label axes and add title.
xlabel('X (mm)');
ylabel('Y (mm)');
title('Trajectory Plot with Reference Line');

% Add legend and enable equal axis scaling for accurate spatial representation.
legend('Location', 'best');
axis equal;  
grid on;

% Set fixed axis limits to focus on the region of interest.
xlim([0, 4500]);
ylim([-600, 300]);

% Release hold on the current axes.
hold off;
```
