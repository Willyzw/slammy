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
bagselect = select(bag, 'Topic', '/odom');
odomdata = readMessages(bagselect);

% initialize plot
figure
hold on
title('Odometry /odom')
ylabel('Y [m]')
xlabel('X [m]')
i=1;
last_time=0;
X=[];
Y=[];

% In loop:
for i=1:1:length(odomdata)
        
        % read the next odometry entry
        x=odomdata{i}.Pose.Pose.Position.X;
        y=odomdata{i}.Pose.Pose.Position.Y;
        
        % append to trajetory
        X=[X;x];
        Y=[Y;y];
        
        % plot
        plot(X,Y,'.-r')
        axis equal
        grid on
        hold off
        drawnow

end
trajectory=[X,Y];

%% save trajectory
%writematrix(trajectory,'trajectory_odometry_slam_two_loops.txt')
%save('trajectory_odometry_slam_two_loops','trajectory')
%saveas(gca,'slammy_example_odometry.jpg')