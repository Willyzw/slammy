% This file is part of Simultaneous Localization and Mapping (SLAM) course
% at the University of Stuttgart held by institute of navigation (ins) and 
% institute for photogrammetry (ifp).
% See https://github.com/Willyzw/slammy for full details.
% Author: Philipp J. Schneider (ifp), 2021

% run slam


% check if slam exists
% if not initialize
if ~exist('slamObj','var')
    
    last_time=0;
    % init slam
    maxRange = 10; % meters
    map_resolution = 20; % cells per meter
    
    slamObj = lidarSLAM(map_resolution,maxRange);
    slamObj.LoopClosureThreshold = 200;
    slamObj.LoopClosureSearchRadius = 2;
    slamObj.OptimizationInterval=1;
    MovementThreshold =[0.20 0.0873];
    %slamObj.ScanRegistrationMethod = 'PhaseCorrelation';
end


last_time=laserdata.Header.Stamp.Sec; % set last time variable

% convert the scan to a "lidarScan" object
rho=double(laserdata.Ranges);
theta=double([laserdata.AngleMin:laserdata.AngleIncrement:laserdata.AngleMax]');
theta(rho==inf)=[];
rho(rho==inf)=[];

theta(rho<0.3)=[];
rho(rho<0.3)=[];

scan=lidarScan(rho,theta);

% add can to slamObject (this will trigger the actual slam pipe line)
addScan(slamObj,scan);

% get poses and create occ map
[scansSLAM,poses] = scansAndPoses(slamObj);
occMap = buildMap(scansSLAM,poses,map_resolution,maxRange);

%% show Occupancy grid map (occMap)
% figure 
% occMap.show()

