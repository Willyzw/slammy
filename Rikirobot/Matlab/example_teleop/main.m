clc
clear all
close all

%% set up node
try
ipaddress = "http://141.58.125.213:11311";
rosinit(ipaddress)
catch
    disp('Ros is running')
    
end

%% subscribe publish to topics
global handles
% laser
handles.laserSub = rossubscriber("/scan","BufferSize",5);
receive(handles.laserSub,3);

% keyboard (teleop)
velTopic = "/cmd_vel";
handles.velPub = rospublisher(velTopic);

%% display laser
h_fig=figure
set(h_fig,'KeyPressFcn',@teleop);
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
p_car=plot(x_cart,y_cart,'k','LineWidth',3)
q_cart=quiver([0 0],[0 0],[1 0],[ 0 0],0)

% set axis
ylim([-3 3])
xlim([-3 3])
pbaspect([1 1 1])

% initial map
laserdata=receive(handles.laserSub,3);
rho=laserdata.Ranges;
theta=[laserdata.AngleMin:laserdata.AngleIncrement:laserdata.AngleMax]';
theta(rho==inf)=[];
rho(rho==inf)=[];
[map.X,map.Y]=pol2cart(theta,rho);


while true
try
laserdata=receive(handles.laserSub,3);
catch
    disp('timed out')
end

rho=laserdata.Ranges;
theta=[laserdata.AngleMin:laserdata.AngleIncrement:laserdata.AngleMax]';
theta(rho==inf)=[];
rho(rho==inf)=[];
[X,Y]=pol2cart(theta,rho);

map.X=X;
map.Y=Y;
refreshdata
drawnow

end


