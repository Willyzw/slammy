%run_plot
figure(1)
show(slamObj);
hold on
show(slamObj.PoseGraph,'IDs','off' ); 
drawnow
pause(0)