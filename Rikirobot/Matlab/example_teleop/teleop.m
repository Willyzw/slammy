% This file is part of Simultaneous Localization and Mapping (SLAM) course
% at the University of Stuttgart held by institute of navigation (ins) and 
% institute for photogrammetry (ifp).
% See https://github.com/Willyzw/slammy for full details.
% Author: Philipp J. Schneider (ifp), 2021

function myfun(src,event)
   %create velMSG
   global handles
   velMsg=rosmessage(handles.velPub);
   
   % get the pressed key and assign X velocity and Z angle
   disp(event.Key);    
   switch event.Key
            case 'numpad1'
            % fill velMsg
            velMsg.Linear.X=-0.3;
            velMsg.Angular.Z=0.5;
            case 'numpad2'
            % fill velMsg
            velMsg.Linear.X=-0.3;
            velMsg.Angular.Z=0.0;
            case 'numpad3'
            % fill velMsg
            velMsg.Linear.X=-0.3;
            velMsg.Angular.Z=-0.5;
            case 'numpad4'
            % fill velMsg
            velMsg.Linear.X=0.0;
            velMsg.Angular.Z=0.5;
            case 'numpad5'
            % fill velMsg
            velMsg.Linear.X=0.0;
            velMsg.Angular.Z=0.0;
            case 'numpad6'
            % fill velMsg
            velMsg.Linear.X=0.0;
            velMsg.Angular.Z=-0.5;
            case 'numpad7'
            % fill velMsg
            velMsg.Linear.X=0.3;
            velMsg.Angular.Z=0.5;
            case 'numpad8'
            % fill velMsg
            velMsg.Linear.X=0.3;
            velMsg.Angular.Z=0.0;
            case 'numpad9'
            % fill velMsg
            velMsg.Linear.X=0.3;
            velMsg.Angular.Z=-0.5;
            
   end
       
   
% send msg
send(handles.velPub,velMsg);

end