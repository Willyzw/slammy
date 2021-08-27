%% Script to extract stereo images from a rosbag file
clc
clear
close all

%% 
sides = ["left", "right"];
bagname = "2021-07-16-15-30-46_two_loops_camera";
bag=rosbag('D:\Datasets\rikirobot\' + bagname + '.bag');
output = 'D:\Datasets\rikirobot';

for side = sides
    img_message=select(bag,'Topic','/zed2/zed_node/'+side+'/image_rect_gray');
    data=readMessages(img_message);

    mkdir(output+'/'+bagname+'/'+side)
    filename = output+'/'+bagname+'/'+side+'/%d%09d.png';
    for i=1:img_message.NumMessages
        image = readImage(data{i,1});
        imshow(image);
        stamp = data{i,1}.Header.Stamp;
        imwrite(image,sprintf(filename, stamp.Sec, stamp.Nsec));
    end
end