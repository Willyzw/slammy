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
slamObj.LoopClosureThreshold = 200;
slamObj.LoopClosureSearchRadius = 2;
slamObj.OptimizationInterval=1;

i=1;
last_time=0;
while true
    i=i+1;
    laserdata=laserdata_bag{i};

    
    if (laserdata.Header.Stamp.Sec-last_time)<1
        %last_time=laserdata{i}.Header.Stamp.Sec
        
    else
        run_slam;
    end
    
    
    %% plot
    run_plot;

end