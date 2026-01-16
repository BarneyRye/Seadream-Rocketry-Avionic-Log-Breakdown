%       Code to Read and Plot Data from an extended Kalman Filter
%       Plots a 3D Graph of trajectory
%       File Produced from .C file found in the same GitHub Repo

clear all;
close all;
clc;

flag = 0; %File name found flag
index = 1;

% Find EKF and Raw CSV files
while flag == 0 
    if index > 255
        fprintf("ERROR: File not found\n")
        return;
    end
    EKF_filename = sprintf('ekf%d.csv', index);
    Raw_filename = sprintf('raw%d.csv', index);
    if isfile(EKF_filename) && isfile(Raw_filename)
        flag = 1;
    else
        index = index + 1;
    end
end

% Read CSV data (skip 1 header line)
data = readmatrix(EKF_filename, 'NumHeaderLines', 1);
rawData = readmatrix(Raw_filename, 'NumHeaderLines', 1);

% Extract X, Y, Z positions
X = data(:,7);
Y = data(:,8);
Z = data(:,9);

% Extract time
time = rawData(:,1);

% Plot trajectory colored by time
figure;
scatter3(X, Y, Z, 20, time, 'filled'); % time-colored points
hold on;
colormap(jet);
colorbar;
grid on;
axis equal;
view(45, 30);

% Plot start and end points on top
plot3(X(1), Y(1), Z(1), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g'); % Start
plot3(X(end), Y(end), Z(end), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r'); % End

xlabel('X [m]');
ylabel('Y [m]');
zlabel('Z [m]');
title('3D Trajectory colored by time');
