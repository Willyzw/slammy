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




% import Ground truth
poses_gt = importfile('../../../Groundtruth/slammy_ground_truth_two_loops.txt')
% millimeter to meter
poses_gt(:,1:2)=poses_gt(:,1:2)/1000;


% translate and rotated poses_slam by the start pose of gt
poses_slam(:,1)=poses_slam(:,1)+poses_gt(1,1);
poses_slam(:,2)=poses_slam(:,2)+poses_gt(1,2);
poses_slam(:,3)=poses_slam(:,3)+poses_gt(1,3);

% plot
figure
plot(poses_slam(:,1),poses_slam(:,2),'.-r')
hold on
plot(poses_gt(:,1),poses_gt(:,2),'.-k')
plot(trajectory(:,1),trajectory(:,2),'.-b')




legend('Lidar SLAM','GT','Odometry')
axis equal
grid on
saveas(gca,'example_compare_trajectories.jpg')