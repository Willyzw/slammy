% run slam
 

% check if slam exists
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

% update last slam time   
   % check time since last slam
    if (laserdata.Header.Stamp.Sec-last_time)>1
        %last_time=laserdata{i}.Header.Stamp.Sec
         %% run slam
 

   % check time since last slam

   last_time=laserdata.Header.Stamp.Sec;
    
    rho=double(laserdata.Ranges);
    theta=double([laserdata.AngleMin:laserdata.AngleIncrement:laserdata.AngleMax]');
    theta(rho==inf)=[];
    rho(rho==inf)=[];
    
    theta(rho<0.3)=[];
    rho(rho<0.3)=[];
    
    scan=lidarScan(rho,theta);
    addScan(slamObj,scan);
    
    % get poses and create occ map
    [scansSLAM,poses] = scansAndPoses(slamObj);
    occMap = buildMap(scansSLAM,poses,map_resolution,maxRange);
       end
