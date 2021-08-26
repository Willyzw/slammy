close all
clear all

%% settings

% load bag
bag = rosbag('../../rosbag/two_loops_robot/2021-07-16-15-31-07_two_loops_robot.bag');
bSel = select(bag,'Topic','/scan');

% start point in time in bag
time_start = 1;

% time step width
delta = 2;

% end point in time in bag
time_end = bSel.NumMessages;

% icp inlier ratio
icpInlierRatio = 0.65;

%% loop closure settings

% maximum distance of loop closure candidates from current node
distForLoopClosure = 1.0;

% maximum icp matching error for loop closure
maxErrorLoopClosure = 0.1;

% grid size for downsampling
dsGrid = 0.05;

% angle difference when a new key frame is generated
angleDiffNewKey = 10*pi/180;

% distance when a new key frame is generated
translDiffNewKey = 0.3;

% number of loop closure conditions per node
maxNumLoopClosureEdgesPerNode = 3; % 1 ?

% minimum difference of current node id to potential loop closure candidate
% nodes
idDiffToCurrKeyFrameId = 2; % 50 ?

%% init stuff
PC_new = [];
pg = poseGraph;

PoseTimes = zeros( round( (time_end-time_start)/delta +delta), 1);
PoseTimes(1) = 1;

id = 1;
eul=[0,0,0];

%% loop over rosbag
for t = time_start : delta : time_end
    
    % read scan from rosbag
    ScanMsg = readMessages(bSel,t);
    
    % convert to cartesian coordinates
    cart = readCartesian(ScanMsg{1});
    
    % create point cloud
    PC_new = pointCloud([cart,zeros(size(cart,1),1)]);
    PC_new = pcdownsample(PC_new,'gridAverage',dsGrid);
    
    if t == time_start
        PC_old = PC_new;
        tform = rigid3d(eul2rotm([0,0,0],'XYZ'),[0,0 ,0]);
        continue;
    end
    
    % register pcs with icp
    [tformNew,movingReg,rmse] = pcregistericp(PC_new,PC_old,'MaxIterations',100,'Tolerance',[0.001,0.001],'InitialTransform',tform,'InlierRatio',icpInlierRatio);
    disp(rmse);
    if rmse > 0.35 || norm(tform.Translation(1:2))> 0.5
        disp('Error');
    else
        tform=tformNew;
    end
    eul = rotm2eul(tform.Rotation,'XYZ');
    
    % check if current scan is new keyframe
    if  norm(tform.Translation(1:2)) > translDiffNewKey || abs(eul(3) ) > angleDiffNewKey
        
        %% add keyframe
        addRelativePose(pg,[tform.Translation(1:2),-eul(3)],[1 0 0 1 0 1]);
        id = id + 1;
        tform = rigid3d(eul2rotm([0,0,0],'XYZ'),[0,0 ,0]);
        
        % update old pc
        PC_old = PC_new;
        
        % add time stamp
        PoseTimes(id) = t;
        
        %% check for loop closures
        lastKeyPose = nodeEstimates(pg,pg.NumNodes);
        
        Poses = nodeEstimates(pg);
        diffMat = Poses(:,1:2) - lastKeyPose(1:2);
        diffVec = sqrt(diffMat(:,1).^2+diffMat(:,2).^2);
        [val,tmpId] = min(diffVec);
        
        % find potential candidates
        loopClosureCandidates = find(diffVec < distForLoopClosure);
        [~,I] = sort(diffVec(loopClosureCandidates));
        loopClosureCandidates = loopClosureCandidates(I);
        
        numLC = 0;
        
        if isempty(loopClosureCandidates) == 0
            
            % go through candidates
            disp([num2str(numel(loopClosureCandidates)), ' loop closure candidate']);
            
            for k = 1 : numel(loopClosureCandidates)
                
                % dont compare with current scan and scan before
                if loopClosureCandidates(k) > id - idDiffToCurrKeyFrameId
                    continue;
                end
                
                % load candidates pointcloud
                % read scan from rosbag
                ScanMsg = readMessages(bSel,PoseTimes(loopClosureCandidates(k)) );
                
                % convert to cartesian coordinates
                cart = readCartesian(ScanMsg{1});
                
                % create point cloud
                PC_candidate = pointCloud([cart,zeros(size(cart,1),1)]);
                PC_candidate = pcdownsample(PC_candidate,'gridAverage',dsGrid);
                
                % register pcs with icp
                [tformLC,movingReg,rmse] = pcregistericp(PC_candidate,PC_new,'MaxIterations',200,'Tolerance',[0.001,0.001],'InlierRatio',icpInlierRatio);
                disp(rmse);
                eul = rotm2eul(tformLC.Rotation,'XYZ');
                
                if rmse < maxErrorLoopClosure
                    addRelativePose(pg,[tformLC.Translation(1:2),-eul(3)],[1 0 0 1 0 1],id,loopClosureCandidates(k) );
                    numLC = numLC + 1;
                end
                
                % limit loop closure edges per scan
                if numLC >= maxNumLoopClosureEdgesPerNode
                    break;
                end

            end
            
            % optimize
            pg = optimizePoseGraph(pg);
            
        end
        
    end
    
    % plot
    show(pg);
    axis([-4,8,-4,8]);
    pause(0.01);
end

show(pg,'IDs','off');
updatedPG = optimizePoseGraph(pg);
show(updatedPG);

%% create global pointcloud for visualization
figure
for k = 1 : pg.NumNodes
    
    % get pose
    Pose = nodeEstimates(pg,k);
    t = PoseTimes(k);
    
    % read scan from rosbag
    ScanMsg = readMessages(bSel,t);
    
    % convert to cartesian coordinates
    cart = readCartesian(ScanMsg{1});
    
    R = eul2rotm([0,0,Pose(3)],'XYZ');
    R_2d = R(1:2,1:2);
    
    pc_global = (R_2d * cart')' + Pose(1:2);
    
    plot(pc_global(:,1),pc_global(:,2),'.','color','b');
    hold on
    pause(0.01);
    
end

hold on
Poses = nodeEstimates(pg);
plot(Poses(:,1),Poses(:,2),'color','r');

% save poses
save final_poses_graph_icp.mat Poses



