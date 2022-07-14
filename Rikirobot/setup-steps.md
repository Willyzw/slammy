# Stm32 doc: https://github.com/ykevin/rikirobot_docs/tree/master/Stm32
- sudo apt-get install -y git build-essential gcc-arm-none-eabi libstdc++-arm-none-eabi-newlib

- mkdir Projects & cd Projects
````
git clone https://github.com/texane/stlink.git
cd stlink
git checkout v1.6.0
make
cd build/Release
sudo make install
````

- Please make sure which usb is which
````
sudo chmod 666 /dev/ttyUSB0
sudo chmod 666 /dev/ttyUSB1
````

- rosrun rosserial_python serial_node.py /dev/ttyUSB1

# Ps4 controller: http://wiki.ros.org/ds4_driver
- Commands to install driver and ros wrapper
````
git clone https://github.com/naoki-mizuno/ds4drv --branch devel
cd ds4drv
python2 setup.py install --prefix ~/.local
sudo cp udev/50-ds4drv.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
cd ~/catkin_ws/src
git clone https://github.com/naoki-mizuno/ds4_driver.git
git checkout v0.2.0
catkin build ds4_driver
````

- Bluetoothctl to pair and connect controller

- `roslaunch ds4_driver ds4_twist.launch dof:=2`


