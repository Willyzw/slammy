clc
clear all
close all


% load ros bag
bag=rosbag('../../rosbag/two_loops_robot/2021-07-16-15-31-07_two_loops_robot.bag');
bagselect = select(bag, 'Topic', '/odom');
odomdata = readMessages(bagselect);

figure
hold on
title('Odometry /odom')
ylabel('Y [m]')
xlabel('X [m]')
i=1;
last_time=0;
X=[];
Y=[];
for i=1:1:length(odomdata)
 
        x=odomdata{i}.Pose.Pose.Position.X;
        y=odomdata{i}.Pose.Pose.Position.Y;
        
        X=[X;x];
        Y=[Y;y];
        plot(X,Y,'.-r')
        axis equal
        grid on
        hold off
        drawnow

end
trajectory=[X,Y];
%save trajectory
writematrix(trajectory,'trajectory_odometry_slam_two_loops.txt')
save('trajectory_odometry_slam_two_loops','trajectory')

saveas(gca,'slammy_example_odometry.jpg')