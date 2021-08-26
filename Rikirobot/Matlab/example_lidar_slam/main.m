% This file is part of Simultaneous Localization and Mapping (SLAM) course
% at the University of Stuttgart held by institute of navigation (ins) and 
% institute for photogrammetry (ifp).
% See https://github.com/Willyzw/slammy for full details.
% Author: Philipp J. Schneider (ifp), 2021

clc
clear all
close all


% load ros bag
bag=rosbag('../../rosbag/two_loops_robot/2021-07-16-15-31-07_two_loops_robot.bag');
bagselect = select(bag, 'Topic', '/scan');
laserdata_bag = readMessages(bagselect);

% init slam
maxRange = 10; % meters
map_resolution = 20; % cells per meter in Occupancy grid map

slamObj = lidarSLAM(map_resolution,maxRange); 
slamObj.LoopClosureThreshold = 150; % when is a loop closure accepted
slamObj.LoopClosureSearchRadius = 3; % where can a loop closure be
slamObj.OptimizationInterval=10; % after how many loop closures optemize the graph

i=1;
last_time=0;
for i=1:length(laserdata_bag)
    laserdata=laserdata_bag{i};
    
    % check if last slam was more than a second ago
    if (laserdata.Header.Stamp.Sec-last_time)<1
        continue
    else
        
        tic %get timing for slam
        run_slam;
        toc
        %% plot
        run_plot;
    end
    
    
    
    
end

% rad to deg
poses(:,3)=poses(:,3)*180/pi;

%save trajectory
writematrix(poses,'poses_lidar_slam_two_loops.txt')
save('poses_lidar_slam_two_loops','poses','slamObj')