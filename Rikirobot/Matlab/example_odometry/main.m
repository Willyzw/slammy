clc
clear all
close all


% load ros bag
bag=rosbag('../../rosbag/two_loops_robot/2021-07-16-15-31-07_two_loops_robot.bag');
bagselect = select(bag, 'Topic', '/odom');
odomdata = readMessages(bagselect);

figure
hold on
title('Odometry \\odom')
ylabel('Y [m]')
xlabel('X [m]')
i=1;
last_time=0;
X=0
Y=0
while true
    i=i+1;
        x=odomdata{i}.Pose.Pose.Position.X;
        y=odomdata{i}.Pose.Pose.Position.Y;
        
        X=x;
        Y=y;
        plot(X,Y,'.-r')
        axis equal
        grid on
        drawnow

end

saveas(gca,'slammy_example_odometry.jpg')