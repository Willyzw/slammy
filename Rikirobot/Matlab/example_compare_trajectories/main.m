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
poses_odometry = trajectory;

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

% translate and rotated poses_slam by the start pose of gt
poses_lidar_slam(:,1)=poses_lidar_slam(:,1)+poses_gt(1,1);
poses_lidar_slam(:,2)=poses_lidar_slam(:,2)+poses_gt(1,2);
poses_lidar_slam(:,3)=poses_lidar_slam(:,3)+poses_gt(1,3);

% translate and rotated poses of icp graph slam by the start pose of gt
Poses(:,1)=Poses(:,1)+poses_gt(1,1);
Poses(:,2)=Poses(:,2)+poses_gt(1,2);
Poses(:,3)=Poses(:,3)+poses_gt(1,3);

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


%% Joint figure
figure('Position', [10 10 1200 600]);
t = tiledlayout(1,3,'Padding','tight');
axis_limits = [-0.5, 4.0, -0.5, 6];

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
xlabel('a)','fontsize',14)

nexttile
plot(poses_gt(:,1),poses_gt(:,2),'.-k')
hold on
plot(poses_lidar_slam(:,1),poses_lidar_slam(:,2),'.-r', 'markerSize', 3)
plot(poses_icp_graph_slam(:,1),poses_icp_graph_slam(:,2),'.-', 'markerSize', 3, 'color', [0.3010 0.7450 0.9330])
% legend('GT','Lidar SLAM','Lidar ICP Graph SLAM','Location','eastoutside'	)
plot(poses_gt(1,1), poses_gt(1,2), 'b>', 'markerSize', 2, 'LineWidth',3)
plot(poses_gt(length(poses_gt),1), poses_gt(length(poses_gt),2), 'rv', 'markerSize', 2, 'LineWidth',3)
axis equal
axis(axis_limits)
grid on
xlabel('b)','fontsize',14)

nexttile
plot(poses_gt(:,1),poses_gt(:,2),'.-k')
hold on
plot([-5,-5],'.-b')
plot([-5,-5],'.-r')
plot([-5,-5],'.-', 'color', [0.3010 0.7450 0.9330])

plot(poses_orb_slam(:,1), poses_orb_slam(:,2),'.-', 'markerSize', 1, 'Color', [0 0.4470 0.7410])
plot(poses_stereo_dso(:,1), poses_stereo_dso(:,2),'.-g', 'markerSize', 1)
plot(poses_droid_mono(:,1), poses_droid_mono(:,2),'.-', 'markerSize', 1,'Color',[0.8500 0.3250 0.0980])
plot(poses_droid_rgbd(:,1), poses_droid_rgbd(:,2),'.-m', 'markerSize', 1)

plot(poses_gt(1,1), poses_gt(1,2), 'b>', 'markerSize', 2, 'LineWidth',3)
plot(poses_gt(length(poses_gt),1), poses_gt(length(poses_gt),2), 'rv', 'markerSize', 2, 'LineWidth',3)
legend('GT','Wheel Odom','Lidar SLAM','Lidar ICP Graph SLAM', 'Stereo ORB-SLAM','Stereo DSO','DROID mono*','DROID rgbd','Start', 'End','Location','eastoutside')
axis equal
axis(axis_limits)
grid on
xlabel('c)','fontsize',14)
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