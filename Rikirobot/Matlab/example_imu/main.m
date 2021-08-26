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
bagselect = select(bag, 'Topic', '/imu/data');
imudata = readMessages(bagselect);

t(1)= datetime(1970,1,1)+seconds(imudata{1}.Header.Stamp.Sec  )+seconds(imudata{1}.Header.Stamp.Nsec*10e-9  )


for i=1:1385
    
    avx(i)=imudata{i}.AngularVelocity.X;
    avy(i)=imudata{i}.AngularVelocity.Y;
    avz(i)=imudata{i}.AngularVelocity.Z;
    lax(i)=imudata{i}.LinearAcceleration.X;
    lay(i)=imudata{i}.LinearAcceleration.Y;
    laz(i)=imudata{i}.LinearAcceleration.Z;
    

    t(i)= datetime(1970,1,1)+seconds(imudata{i}.Header.Stamp.Sec  )+seconds(imudata{i}.Header.Stamp.Nsec*1e-9  );
    
    i
end

figure
title('Inertial Navigation \\imu\\data')
hold on

    subplot(2,1,1)
    plot(t,avx,'-r')
    hold on
    plot(t,avy,'-g')
    plot(t,avz,'-b')
        title('Angular Velocity')

    legend('X','Y','Z')
    xlabel('t [s]')
    ylabel(' \omega [rad/s]')
    
    subplot(2,1,2)
    
    plot(t,lax,'-r')
    hold on
    plot(t,lay,'-g')
    plot(t,laz,'-b')
        legend('X','Y','Z')

    xlabel('t [s]')
    ylabel('a [m/s]')
    title('Linear Acceleration')
    grid on
    drawnow
    
saveas(gca,'slammy_example_imu.jpg')