# slammy repository
<img src="assets/slammy_portrait.jpg" alt="slammy_portrait" width="500"/>


This repository contains stuffs for the slam course at the University of Stuttgart. Three parts are
currently available:
- Rikirobot + Lidar
- Jetson Xavier NX board + Zed2 camera
- Gopro for a birdview groundtruth


## Rikitrobot + lidar
More details see [Readme.md](Rikirobot/README.md)

Details about the rover such as:
* How to get started with the rover
* How to control the rover
* How to record a rosbag (including the "two_loops" rosbag)
* How to play the rosbag in matlab
* Several examples:
 * 2d lidar slam
 * Odometry
 * Inertial navigation data


<img src="Rikirobot\Matlab\example_lidar_slam\slammy_example_lidar_slam.jpg" alt="slammy_example_lidar_slam.jpg"  /> </br>


## Jetson Xavier NX board + Zed2 camera
More details see [Readme.md](Jetson_NX_Zed2/README.md)

Details about the stereo camera such as:
* How to get started with the camera/ jetson Xavier
* A rosbag of the "two_loops" run
* Examples of mono/binocular ORB slam in Matlab


<img src="Jetson_NX_Zed2/assets/slammy_zed_two_loops.gif" alt="slammy_zed_two_loops.gif"  /> </br>


<img src="Jetson_NX_Zed2/assets/slammy_zeb_orb_two_loops.gif" alt="slammy_zeb_orb_two_loops.gif"  /> </br>

## Ground truth
More details see [Readme.md](Groundtruth/README.md)

The ground truth was captured via a GoPro, by tracking an ArUco marker's pose.

<img src="Groundtruth/assets/slammy_ground_truth_two_loops.gif" alt="slammy_ground_truth_two_loops.gif"  /> </br>
Video: [Groundtruth/assets/slammy_ground_truth_two_loops.avi](Groundtruth/assets/slammy_ground_truth_two_loops.avi)
