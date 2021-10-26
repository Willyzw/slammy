% This file is part of Simultaneous Localization and Mapping (SLAM) course
% at the University of Stuttgart held by institute of navigation (ins) and 
% institute for photogrammetry (ifp).
% See https://github.com/Willyzw/slammy for full details.
% Author: Philipp J. Schneider (ifp), 2021

clc
clear all
close all

%% set up node and connect to Master
rosshutdown

ipaddress = "http://141.58.125.219:11311";
rosinit(ipaddress)

pause(1)
%% subscribe publish to topics
% laser
laserSub = rossubscriber("/scan","BufferSize",10);
receive(laserSub,10);


%% Set up the plot for the laser data:
h_fig=figure;
% plot /scan
map.X=[0];
map.Y=[0];
p_scan=plot(map.X,map.Y,'.');
p_scan.XDataSource = 'map.X';
p_scan.YDataSource = 'map.Y';
% plot car body and direction
hold on
y_cart=[-0.1 0.1 0.1 -0.1 -0.1];
x_cart=[0.1 0.1 -0.15 -0.15 0.1];
p_car=plot(x_cart,y_cart,'k','LineWidth',3);
q_cart=quiver([0 0],[0 0],[1 0],[ 0 0],0);
% set axis
ylim([-3 3])
xlim([-3 3])
pbaspect([1 1 1])

%% initial map
laserdata=receive(laserSub,5);
rho=laserdata.Ranges;
theta=[laserdata.AngleMin:laserdata.AngleIncrement:laserdata.AngleMax]';
theta(rho==inf)=[];
rho(rho==inf)=[];
[map.X,map.Y]=pol2cart(theta,rho);
refreshdata
drawnow
%% In Loop:
while true
    
try % try if new data is available 
laserdata=receive(laserSub,5);
catch % if no instead of error display time out'
    disp('timed out')
end

% Calculate points from ranges and angles
rho=laserdata.Ranges;
theta=[laserdata.AngleMin:laserdata.AngleIncrement:laserdata.AngleMax]'; % create an angle list
theta(rho==inf)=[]; % delete empty ones
rho(rho==inf)=[];
[X,Y]=pol2cart(theta,rho); % polar to cartesian coordinates

% update plot
map.X=X;
map.Y=Y;
refreshdata
drawnow

end


