% This file is part of Simultaneous Localization and Mapping (SLAM) course
% at the University of Stuttgart held by institute of navigation (ins) and 
% institute for photogrammetry (ifp).
% See https://github.com/Willyzw/slammy for full details.
% Author: Philipp J. Schneider (ifp), 2021

clc
clear all
close all

%% Load GT and odometer trajectory
% import Ground truth
poses_gt = importfile('../../../Groundtruth/slammy_ground_truth_two_loops.txt');
% millimeter to meter
poses_gt(:,1:2)=poses_gt(:,1:2)/1000;

% import trajectory from odometry
load('../example_odometry/trajectory_odometry_slam_two_loops.mat')
poses_odometry = trajectory*0.90;
% poses_odometry = trajectory;

% plot
% figure
% plot(poses_gt(:,1),poses_gt(:,2),'.-k')
% hold on
% plot(poses_odometry(:,1),poses_odometry(:,2),'.-b', 'markerSize', 2)
% legend('GT','Wheel Odometer','Location','eastoutside')
% axis equal
% axis([-1, 5, -1, 7])
% grid on
% exportgraphics(gca,'example_compare_trajectories_odom.png','Resolution',300)

%% Load lidar trajectories
% import poses from lidar slam
load('../example_lidar_slam/poses_lidar_slam_two_loops.mat')
poses_lidar_slam=poses;


% import trajectory from icp graph slam
load('../example_icp_graph_slam/final_poses_graph_icp.mat')
poses_icp_graph_slam = Poses;
load('../example_icp_graph_slam/final_poses_graph_icp_deskewed.mat')
poses_icp_graph_slam_deskewed = Poses;

% translate and rotated poses_slam by the start pose of gt
poses_lidar_slam(:,1)=poses_lidar_slam(:,1)+poses_gt(1,1);
poses_lidar_slam(:,2)=poses_lidar_slam(:,2)+poses_gt(1,2);
poses_lidar_slam(:,3)=poses_lidar_slam(:,3)+poses_gt(1,3);

% translate and rotated poses of icp graph slam by the start pose of gt
poses_icp_graph_slam(:,1)=poses_icp_graph_slam(:,1)+poses_gt(1,1);
poses_icp_graph_slam(:,2)=poses_icp_graph_slam(:,2)+poses_gt(1,2);
poses_icp_graph_slam(:,3)=poses_icp_graph_slam(:,3)+poses_gt(1,3);
poses_icp_graph_slam_deskewed(:,1)=Poses(:,1)+poses_gt(1,1);
poses_icp_graph_slam_deskewed(:,2)=Poses(:,2)+poses_gt(1,2);
poses_icp_graph_slam_deskewed(:,3)=Poses(:,3)+poses_gt(1,3);

% plot
% figure
% plot(poses_gt(:,1),poses_gt(:,2),'.-k')
% hold on
% plot(poses_lidar_slam(:,1),poses_lidar_slam(:,2),'.-r', 'markerSize', 3)
% plot(poses_icp_graph_slam(:,1),poses_icp_graph_slam(:,2),'.-m', 'markerSize', 3)
% legend('GT','Lidar SLAM','Lidar ICP Graph SLAM','Location','eastoutside'	)
% axis equal
% axis([-1, 5, -1, 7])
% grid on
% exportgraphics(gca,'example_compare_trajectories_lidar.png','Resolution',300)

%% Compare the trajectory results of vision system
% import vision result
poses_orb_slam = load_and_process_trj('../../../Vision/results/slammy_two_loops_traj_stereo-orb-slam.txt', [4,2], poses_gt);
poses_stereo_dso = load_and_process_trj('../../../Vision/results/slammy_two_loops_traj_stereo-dso.txt', [12,4], poses_gt);
poses_droid_mono = load_and_process_trj('../../../Vision/results/slammy_two_loops_traj_droid_mono.txt', [4,2], poses_gt);
poses_droid_rgbd = load_and_process_trj('../../../Vision/results/slammy_two_loops_traj_droid_rgbd.txt', [4,2], poses_gt);

% Mono SLAM has scale ambiguity, manually found 1.2 a rough estimate 
poses_droid_mono = poses_droid_mono * 1.2;

% plot
% figure
% plot(poses_gt(:,1),poses_gt(:,2),'.-k')
% hold on
% plot(poses_orb_slam(:,1), poses_orb_slam(:,2),'.-r', 'markerSize', 1)
% plot(poses_stereo_dso(:,1), poses_stereo_dso(:,2),'.-g', 'markerSize', 1)
% plot(poses_droid_mono(:,1), poses_droid_mono(:,2),'.-b', 'markerSize', 1)
% plot(poses_droid_rgbd(:,1), poses_droid_rgbd(:,2),'.-m', 'markerSize', 1)
% legend('GT','Stereo ORB-SLAM','Stereo DSO','DROID mono','DROID rgbd','Location','eastoutside')
% axis equal
% axis([-1, 5, -1, 7])
% grid on
% exportgraphics(gca,'example_compare_trajectories_vision.png','Resolution',300)

%% Quantitative result estimate-to-GT
% [dist1, g1] = evalE2G(poses_gt, poses_odometry, 'Odometry');
% [dist2, g2] = evalE2G(poses_gt, poses_lidar_slam(:,[1,2]), 'Lidar-slam');
% [dist3, g3] = evalE2G(poses_gt, poses_icp_graph_slam(:,[1,2]), 'ICP-graph-slam');
% [dist8, g8] = evalE2G(poses_gt, poses_icp_graph_slam_deskewed(:,[1,2]), 'ICP-graph-slam de-skewed');
% [dist6, g6] = evalE2G(poses_gt, poses_stereo_dso, 'Stereo-DSO');
% [dist7, g7] = evalE2G(poses_gt, poses_orb_slam, 'ORB-slam');
% [dist4, g4] = evalE2G(poses_gt, poses_droid_mono, 'DROID-mono');
% [dist5, g5] = evalE2G(poses_gt, poses_droid_rgbd, 'DROID-rgbd');
% 
% dist = [dist1; dist2; dist3; dist8; dist6; dist7; dist4; dist5];
% g = [g1; g2; g3; g8; g6; g7; g4; g5];
% figure
% boxplot(dist, g)
% title('Absolute trajectory error (ATE) Estimate-to-GT')
% ylabel('Error [m]')

%% Quantitative result GT-to-estimate
[dist1, g1] = evalG2E(poses_gt, poses_odometry, 'Wheel Odometry');
[dist2, g2] = evalG2E(poses_gt, poses_lidar_slam(:,[1,2]), 'Matlab-Lidar-slam');
[dist3, g3] = evalG2E(poses_gt, poses_icp_graph_slam(:,[1,2]), 'ICP-graph-slam');
[dist8, g8] = evalG2E(poses_gt, poses_icp_graph_slam_deskewed(:,[1,2]), 'ICP-graph-slam de-skewed');
[dist6, g6] = evalG2E(poses_gt, poses_stereo_dso, 'Stereo-DSO');
[dist7, g7] = evalG2E(poses_gt, poses_orb_slam, 'ORB-slam');
[dist4, g4] = evalG2E(poses_gt, poses_droid_mono, 'DROID-mono*');
[dist5, g5] = evalG2E(poses_gt, poses_droid_rgbd, 'DROID-rgbd');

dist = [dist1; dist2; dist3; dist8; dist6; dist7; dist4; dist5];
g = [g1; g2; g3; g8; g6; g7; g4; g5];

figure('Position', [10 10 800 300]);
boxplot(dist, g, 'Whisker', 1e4)
% title('Absolute trajectory error (ATE) GT-to-Estimate')
ylabel('Translational Error [m]')
exportgraphics(gca, 'ATE_G2E.png', 'Resolution', 600)

%% Joint figure
figure('Position', [10 10 1200 600]);
t = tiledlayout(1,3,'Padding','tight');
t.TileSpacing = 'compact';
t.Padding = 'compact';
axis_limits = [-1, 4.5, -1, 7];

nexttile
plot(poses_gt(:,1),poses_gt(:,2),'.-k')
hold on
plot(poses_odometry(:,1),poses_odometry(:,2),'.-b', 'markerSize', 2)
% legend('GT','Wheel Odometer','Location','eastoutside')
plot(poses_gt(1,1), poses_gt(1,2), 'b>', 'markerSize', 2, 'LineWidth',3)
plot(poses_gt(length(poses_gt),1), poses_gt(length(poses_gt),2), 'rv', 'markerSize', 2, 'LineWidth',3)
axis equal
axis([-1, 4.5, -1, 7])
grid on
xlabel({'\fontsize{10}X[m]','\fontsize{14}a)'})
ylabel('Y[m]','fontsize',10)

nexttile
plot(poses_gt(:,1),poses_gt(:,2),'.-k')
hold on
plot(poses_lidar_slam(:,1),poses_lidar_slam(:,2),'.-r', 'markerSize', 3)
plot(poses_icp_graph_slam(:,1),poses_icp_graph_slam(:,2),'.-', 'markerSize', 3, 'color', [0.3010 0.7450 0.9330])
plot(poses_icp_graph_slam_deskewed(:,1),poses_icp_graph_slam_deskewed(:,2),'.-', 'markerSize', 3, 'color', [0.8 0.54 0.33])
% legend('GT','Lidar SLAM','Lidar ICP Graph SLAM','Location','eastoutside'	)
plot(poses_gt(1,1), poses_gt(1,2), 'b>', 'markerSize', 2, 'LineWidth',3)
plot(poses_gt(length(poses_gt),1), poses_gt(length(poses_gt),2), 'rv', 'markerSize', 2, 'LineWidth',3)
axis equal
axis(axis_limits)
grid on
xlabel({'\fontsize{10}X[m]','\fontsize{14}b)'})
% xlabel('b)','fontsize',14)

nexttile
plot(poses_gt(:,1),poses_gt(:,2),'.-k')
hold on
plot([-5,-5],'.-b')
plot([-5,-5],'.-r')
plot([-5,-5],'.-', 'color', [0.3010 0.7450 0.9330])
plot([-5,-5],'.-', 'color', [0.8 0.54 0.33])

plot(poses_orb_slam(:,1), poses_orb_slam(:,2),'.-', 'markerSize', 1, 'Color', [0 0.4470 0.7410])
plot(poses_stereo_dso(:,1), poses_stereo_dso(:,2),'.-g', 'markerSize', 1)
plot(poses_droid_mono(:,1), poses_droid_mono(:,2),'.-', 'markerSize', 1,'Color',[0.8500 0.3250 0.0980])
plot(poses_droid_rgbd(:,1), poses_droid_rgbd(:,2),'.-m', 'markerSize', 1)

plot(poses_gt(1,1), poses_gt(1,2), 'b>', 'markerSize', 2, 'LineWidth',3)
plot(poses_gt(length(poses_gt),1), poses_gt(length(poses_gt),2), 'rv', 'markerSize', 2, 'LineWidth',3)
legend('GT','Wheel Odometry','Matlab-Lidar-slam','ICP-Graph-slam', 'ICP-Graph-slam de-skewed','Stereo-ORB-SLAM','Stereo-DSO','DROID-mono*','DROID-rgbd','Start', 'End','Location','eastoutside')
axis equal
axis(axis_limits)
grid on
xlabel({'\fontsize{10}X[m]','\fontsize{14}c)'})
exportgraphics(t, 'joint.png', 'Resolution', 600)


%% functions
function poses = load_and_process_trj(filename, xy_indices, poses_gt)
    % import vision result
    poses = importdata(filename);

    % correct coordinate frame from x-right, y-down, z-front 3D coordinate
    % to x-front, y-left 2D coordinate
    poses = poses(:,xy_indices);
    poses(:,2) = -poses(:,2);
    
    % translate and rotated poses_slam by the start pose of gt
    poses(:,1) = poses(:,1)+poses_gt(1,1);
    poses(:,2) = poses(:,2)+poses_gt(1,2);
end

function [dist, label] = evalE2G(poses_gt, poses_est, name)
    [~, dist] = dsearchn(poses_gt(:,[1,2]), poses_est);
    label = repmat({name}, length(dist), 1);
    disp("E2G error " + length(poses_est) + " of " + name + ": " + max(dist) + " RMSE: " + RMSE(dist))
end

function [dist, label] = evalG2E(poses_gt, poses_est, name)
    [~, dist] = dsearchn(poses_est, poses_gt(:,[1,2]));
    label = repmat({name}, length(dist), 1);
    disp("G2E error " + length(poses_est) + " of " + name + ": " + max(dist) + " RMSE: " + RMSE(dist))
end

function rmse = RMSE(errors)
    rmse = sqrt(mean(errors.^2));  % Root Mean Squared Error
end
