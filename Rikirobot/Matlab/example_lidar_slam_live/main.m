% This file is part of Simultaneous Localization and Mapping (SLAM) course
% at the University of Stuttgart held by institute of navigation (ins) and
% institute for photogrammetry (ifp).
% See https://github.com/Willyzw/slammy for full details.
% Author: Philipp J. Schneider (ifp), 2022
clc
clear all
close all
% add path
addpath(fullfile('..','example_lidar_slam'))
%% set up node and connect to Master
rosshutdown
ipaddress = "http://192.168.1.167:11311";
rosinit(ipaddress)
pause(3)
%% subscribe publish to topics
% laser
laserSub = rossubscriber("/scan","BufferSize",10);
laserdata=receive(laserSub,5);
% init slam
maxRange = 10; % meters
map_resolution = 20; % cells per meter in Occupancy grid map
slamObj = lidarSLAM(map_resolution,maxRange);
slamObj.LoopClosureThreshold = 150; % when is a loop closure accepted
slamObj.LoopClosureSearchRadius = 3; % where can a loop closure be
slamObj.OptimizationInterval=10; % after how many loop closures optemize the graph
i=1;
last_time=0;
while true
    laserdata=receive(laserSub,1);
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