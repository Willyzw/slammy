# Rikirobot

For general settings:
https://ubuntu-mate.org/raspberry-pi/


## How to conect to riki robot via ssh


`ssh rikirobot@141.58.125.213`

user: `rikirobot`
pw: `123456`

you might need an ip scanner to find the ip.

if you never connected to your local wifi, use hdmi mouse and keyboard

## Set up the Lidar, odometry and tele op
`./Desktop/run_rikirobo.sh`

## Record a ROS bag
`./record.sh`

## Some use full links

English
https://www.programmersought.com/article/70391490094/

Chinese
https://github.com/ykevin/rikirobot_docs/tree/master/UserManual


## MATLAB examples

### Teleop

[Rikirobot\Matlab\example_teleop](Rikirobot\Matlab\example_teleop)

Remote control via numpad for slammy.
Control the robot by publishing ROS messages with the desired linear and angular velocity.

You need the current ip address.

Use the numpad to navigate:

8: Forward

7: Forward and turn left

9: Forward and turn right

4: Sharp left

2: Backwards

6: Sharp right

1: Backwards and turn left

3: Backwards and turn right


### Pure Lida SLAM from rosbag
[Rikirobot\Matlab\example_lidar_slam](Rikirobot\Matlab\example_lidar_slam)

Use the MATLAB's `lidarSLAM` to evaluate the recorded ROSbag.

The lidarSLAM class performs simultaneous localization and mapping (SLAM) for lidar scan sensor inputs. The SLAM algorithm takes in lidar scans and attaches them to a node in an underlying pose graph. The algorithm then correlates the scans using scan matching. It also searches for loop closures, where scans overlap previously mapped regions, and optimizes the node poses in the pose graph.


<img src="Matlab\example_lidar_slam\slammy_example_lidar_slam.jpg" alt="slammy_example_lidar_slam.jpg"  hight="500"/> </br>


### Odometry
[Rikirobot\Matlab\example_odometry](Rikirobot\Matlab\example_odometry)

Visualize the topic `\odom`, which, contains the accumulated odometry data.

<img src="Matlab\example_odometry\slammy_example_odometry.jpg" alt="slammy_example_odometry.jpg"  hight="500"/> </br>


### Inertial navigation
[Rikirobot\Matlab\example_imu](Rikirobot\Matlab\example_imu)


madgwick filtered https://nitinjsanket.github.io/tutorials/attitudeest/madgwick imu data.

If played from, rosbag an exact timestamp is not available.

<img src="Matlab\example_imu\slammy_example_imu.jpg" alt="slammy_example_imu.jpg"  hight="500"/> </br>
