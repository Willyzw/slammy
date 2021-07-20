# Environment setup
- Flash SD card & first boot configuration e.g. create account
  - Follow https://developer.nvidia.com/embedded/learn/get-started-jetson-xavier-nx-devkit
  - User: `jetson`; PW: `jetson`
  - `ssh jetson@141.58.125.203`

- Install zsh & oh-my-zsh
  - Follow https://github.com/ohmyzsh/ohmyzsh/wiki (Optional)

- Install ZED SDK
  - Follow https://www.stereolabs.com/developers/release/
  - note: `sudo apt update` needed before executing the install script
  - Executables will be added under `/usr/local/bin` and so directly executable

- Install ROS melodic
  - Follow http://wiki.ros.org/melodic/Installation/Ubuntu
  - note `sudo apt install python-rosdep` needed for rosdep
  - `sudo apt install python-catkin-tools` needed for catkin_tools
  - remember to add `source /opt....sh` to ~/.bashrc and ~/.zshrc depending on which shell you use

- Install zed-ros-wrapper& zed-ros-examples
  - Link: https://github.com/stereolabs/zed-ros-wrapper
  - Link: https://github.com/stereolabs/zed-ros-examples
  - note: cloning to catkin directory `~/Softwares/src`
  - Compile with `catkin build`
  - Start Zed node with `roslaunch zed_wrapper zed2.launch` (launch `roscore` if needed)

- Visualization in rviz
  - Command `roslaunch zed_display display_zed2.launch`


# Run ORB-SLAM3
- Configure ORB-slam3
  - Link: `https://github.com/UZ-SLAMLab/ORB_SLAM3.git`
  - Clone to `~/Software/projects/ORB_SLAM3`
  - Follow https://github.com/UZ-SLAMLab/ORB_SLAM3#6-ros-examples to build the ROS node
  - Resolve compile error caused by opencv version https://github.com/UZ-SLAMLab/ORB_SLAM3/issues/300
  - Better limit the number of threads in build.sh to avoid system freezes (e.g. make -j3)

- Adapt camera parameter for Zed2 camera
  - Look up the parameters from `rostopic echo /zed2/zed_node/rgb/camera_info`
  - Note the topic name of the image message also needs to be adapted in the source code.
  - Details please turn to the local changes in the local ORB-SLAM3 repository

- Run ORB-SLAM3 with Zed images
  - Online with streamed images
    - One terminal for zed driver `roslaunch zed_wrapper zed2.launch`
    - Other one for ORB-slam and switch to directory `cd Software/projects/ORB_SLAM3`
    - `rosrun ORB_SLAM3 Mono Vocabulary/ORBvoc.txt Examples/Monocular/Zed2.yaml` (Monocular setting)
    - `rosrun ORB_SLAM3 Stereo Vocabulary/ORBvoc.txt Examples/Stereo/Zed2.yaml 0` (Stereo setting)
  - Offline with a recorded rosbag
    - `roslaunch zed_wrapper zed2.launch` & `rosbag record -a`
    - rosbag play 2021-xx-xxxx.bag /zed/left/image_raw:=/camera/image_raw
    - `rosrun ORB_SLAM3 Mono PATH_TO_VOCABULARY PATH_TO_SETTINGS_FILE`

# Run DSO
- Clone and build DSO project
  - Link: `https://github.com/JakobEngel/dso.git`
  - Clone to `~/Software/projects/dso` and follow the project instruction to build the project

- Build DSO ROS wrapper
  - Link: `https://github.com/BlueWhaleRobot/dso_ros`
  - Clone to catkin workspace `~/Software/src/dso_ros`
  - Set the DSO_PATH before build `export DSO_PATH=/home/jetson/Software/projects/dso`
  - Note this ROS wrapper version contains bugfix therefore we choose this version over the original one

- Run DSO
  - `rosrun dso_ros dso_live image:=/zed2/zed_node/rgb/image_rect_gray calib=/home/jetson/Software/src/dso_ros/camera.txt mode=1`

# Record rosbag file
- Terminal 1: launch the zed2 driver with `roslaunch zed_wrapper zed2.launch`
- Terminal 2: start the recording with `sudo ./data/record.sh`(sudo for correcting the system date)
