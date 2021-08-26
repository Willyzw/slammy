% This file is part of Simultaneous Localization and Mapping (SLAM) course
% at the University of Stuttgart held by institute of navigation (ins) and 
% institute for photogrammetry (ifp).
% See https://github.com/Willyzw/slammy for full details.
% Author: Philipp J. Schneider (ifp), 2021

%run_plot
figure(1)
show(slamObj);
hold on
show(slamObj.PoseGraph,'IDs','off' ); 
drawnow
hold off
pause(0)