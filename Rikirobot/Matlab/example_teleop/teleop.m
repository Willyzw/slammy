function myfun(src,event)
   disp(event.Key);
   %create velMSG
   global handles
    velMsg=rosmessage(handles.velPub);
   switch event.Key
       
            case 'numpad1'
            % fill velMsg
            velMsg.Linear.X=-0.3
            velMsg.Angular.Z=0.5
            case 'numpad2'
            % fill velMsg
            velMsg.Linear.X=-0.3
            velMsg.Angular.Z=0.0
            case 'numpad3'
            % fill velMsg
            velMsg.Linear.X=-0.3
            velMsg.Angular.Z=-0.5
            case 'numpad4'
            % fill velMsg
            velMsg.Linear.X=0.0
            velMsg.Angular.Z=0.5
            case 'numpad5'
            % fill velMsg
            velMsg.Linear.X=0.0
            velMsg.Angular.Z=0.0
            case 'numpad6'
            % fill velMsg
            velMsg.Linear.X=0.0
            velMsg.Angular.Z=-0.5
            case 'numpad7'
            % fill velMsg
            velMsg.Linear.X=0.3
            velMsg.Angular.Z=0.5
            case 'numpad8'
            % fill velMsg
            velMsg.Linear.X=0.3
            velMsg.Angular.Z=0.0
            case 'numpad9'
            % fill velMsg
            velMsg.Linear.X=0.3
            velMsg.Angular.Z=-0.5
            
   end
       
   
% send msg
send(handles.velPub,velMsg);

end