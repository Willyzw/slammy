% This file is part of Simultaneous Localization and Mapping (SLAM) course
% at the University of Stuttgart held by institute of navigation (ins) and 
% institute for photogrammetry (ifp).
% See https://github.com/Willyzw/slammy for full details.
% Author: Philipp J. Schneider (ifp), 2021

clc
clear all
close all


% import poses from slam
load('../example_lidar_slam/poses_lidar_slam_two_loops.mat')
poses_slam=poses;

% import trajectory from odometry
load('../example_odometry/trajectory_odometry_slam_two_loops.mat')

% import trajectory from icp graph slam
load('../example_icp_graph_slam/final_poses_graph_icp.mat')

% import Ground truth
poses_gt = importfile('../../../Groundtruth/slammy_ground_truth_two_loops.txt');
% millimeter to meter
poses_gt(:,1:2)=poses_gt(:,1:2)/1000;

% translate and rotated poses_slam by the start pose of gt
poses_slam(:,1)=poses_slam(:,1)+poses_gt(1,1);
poses_slam(:,2)=poses_slam(:,2)+poses_gt(1,2);
poses_slam(:,3)=poses_slam(:,3)+poses_gt(1,3);

% translate and rotated poses of icp graph slam by the start pose of gt
Poses(:,1)=Poses(:,1)+poses_gt(1,1);
Poses(:,2)=Poses(:,2)+poses_gt(1,2);
Poses(:,3)=Poses(:,3)+poses_gt(1,3);

% plot
figure
plot(poses_slam(:,1),poses_slam(:,2),'.-r')
hold on
plot(poses_gt(:,1),poses_gt(:,2),'.-k')
plot(trajectory(:,1),trajectory(:,2),'.-b')
plot(Poses(:,1),Poses(:,2),'.-m')

legend('Lidar SLAM','GT','Odometry','Lidar ICP Graph SLAM','Location','southoutside'	)
axis equal
grid on
%saveas(gca,'example_compare_trajectories.jpg')

%% Compare the trajectory results of vision system
% import vision result
poses_orb_slam = load_and_process_trj('../../../Vision/results/slammy_two_loops_traj_stereo-orb-slam.txt', [4,2], poses_gt);
poses_stereo_dso = load_and_process_trj('../../../Vision/results/slammy_two_loops_traj_stereo-dso.txt', [12,4], poses_gt);
poses_droid_mono = load_and_process_trj('../../../Vision/results/slammy_two_loops_traj_droid_mono.txt', [4,2], poses_gt);
poses_droid_rgbd = load_and_process_trj('../../../Vision/results/slammy_two_loops_traj_droid_rgbd.txt', [4,2], poses_gt);

% fix unknown scale in mono case
poses_droid_mono = poses_droid_mono * 1.2;

% plot
figure
plot(poses_gt(:,1),poses_gt(:,2),'.-k')
hold on
plot(poses_orb_slam(:,1), poses_orb_slam(:,2),'.-r')
plot(poses_stereo_dso(:,1), poses_stereo_dso(:,2),'.-g')
plot(poses_droid_mono(:,1), poses_droid_mono(:,2),'.-b')
plot(poses_droid_rgbd(:,1), poses_droid_rgbd(:,2),'.-m')
legend('GT','Stereo ORB-SLAM','Stereo DSO','DROID mono','DROID rgbd','Location','eastoutside')
axis equal
grid on
saveas(gca,'../../../Vision/results/example_compare_trajectories_vision.jpg')

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
