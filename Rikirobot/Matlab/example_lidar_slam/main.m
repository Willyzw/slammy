clc
clear all
close all


% load ros bag
bag=rosbag('../../rosbag/two_loops_robot/2021-07-16-15-31-07_two_loops_robot.bag');
bagselect = select(bag, 'Topic', '/scan');
laserdata_bag = readMessages(bagselect);

% init slam
maxRange = 10; % meters
map_resolution = 20; % cells per meter

slamObj = lidarSLAM(map_resolution,maxRange);
slamObj.LoopClosureThreshold = 150;
slamObj.LoopClosureSearchRadius = 3;
slamObj.OptimizationInterval=1;

i=1;
last_time=0;
for i=1:length(laserdata_bag)
    laserdata=laserdata_bag{i};
    
    % check if last slam was more than a second ago
    if (laserdata.Header.Stamp.Sec-last_time)<1
        continue
    else
        tic
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