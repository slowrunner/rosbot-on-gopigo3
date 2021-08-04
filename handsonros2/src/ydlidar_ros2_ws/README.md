Setup YDLIDAR X4 on GoPiGo3 ROS2 ROSbot Dave

=== Prereqs  
- Linux OS: Using Ubuntu 20.04 Server LTS 64-bit  
- External 5v power connected to microUSB of Interface Card  
- USB A to USB C cable from RPi to Interface Card  

==== Download and Install Latest SDK  
```
$ cd ~  (/home/pi)  
$ git clone https://github.com/YDLIDAR/YDLidar-SDK
$ cd YDLidar-SDK/build
$ cmake ..
$ make
$ sudo make install
-- Installing: /usr/local/bin/ydlidar_test     <-- only applicable test for YDLIDAR X4
...

=== RUN YDLIDAR TEST===  
```
/usr/bin/ydlidar_test
- Baudrate? 1 (128000)
- one-way? no
- scan freq? 5



\ \ / /  _ \| |   |_ _|  _ \  / \  |  _ \   
 \ V /| | | | |    | || | | |/ _ \ | |_) |  
  | | | |_| | |___ | || |_| / ___ \|  _ <  
  |_| |____/|_____|___|____/_/   \_\_| \_\  

Baudrate:  
0. 115200  
1. 128000
2. 153600
3. 230400
4. 512000
Please select the lidar baudrate:1
Whether the Lidar is one-way communication[yes/no]:no
Please enter the lidar scan frequency[5-12]:5
YDLidar SDK initializing
YDLidar SDK has been initialized
[YDLIDAR]:SDK Version: 1.0.3
LiDAR successfully connected
[YDLIDAR]:Lidar running correctly ! The health status: good
[YDLIDAR] Connection established in [/dev/ttyUSB0][128000]:
Firmware version: 1.10
Hardware version: 1
Model: X4
Serial: 2020062200002315
LiDAR init success!
[YDLIDAR3]:Fixed Size: 1020
[YDLIDAR3]:Sample Rate: 5K
[YDLIDAR INFO] Current Sampling Rate : 5K
[YDLIDAR INFO] Now YDLIDAR is scanning ......
Scan received[1624198712776831000]: 567 ranges is [8.833922]Hz
Scan received[1624198712898864000]: 572 ranges is [8.756567]Hz
Scan received[1624198713013605000]: 571 ranges is [8.771930]Hz
^C
signal_handler(2)
281473638449600 thread has been canceled
[YDLIDAR INFO] Now YDLIDAR Scanning has stopped ......

```

=== Power Measurements  
LIDAR draws   
- 132mA at battery 12v motor off, RPi shutdown (150mA at 11v)  
- 12mA at battery 11v motor off, RPi and GoPiGo3 up (after ran ydlidar_test)  
- 55mA at 11v scanning at 5 Hz  
565 at 10.2 running  
500 at 10.2 not running  


=== ROS2 node  

Following: README at https://github.com/YDLIDAR/ydlidar_ros2_driver  
see also: https://www.programmersought.com/article/88267580407/  

- Bring down the ROS2 node software  
```
cd ~/rosbot-on-gopigo3/handsonros/src
git clone https://github.com/YDLIDAR/ydlidar_ros2_driver.git ydlidar_ros2_ws/src/ydlidar_ros2_driver
```

- Build the node
```
colcon build --symlink-install --packages-select ydlidar_ros2_driver

NOTE: Initially failed with "1 package has stderr output: ydlidar_ros2_driver",  
      but after running "ros2 doctor", build succeeded
```

- Configure:
```
cp src/ydlidar_ros2_ws/src/ydlidar_ros2_driver/params/ydlidar.yaml src/ydlidar_ros2_ws/src/ydlidar_ros2_driver/params/ydlidar.yaml.orig

nano src/ydlidar_ros2_ws/src/ydlidar_ros2_driver/params/ydlidar.yaml
changes:
  baudrate: 128000
  ...
  sample_rate: 5


nano src/ydlidar_ros2_ws/src/ydlidar_ros2_driver/launch.ydlidar_launch.py
  Duplicate and comment out driver_node declaration,
  Change to: 

    driver_node = LifecycleNode(package='ydlidar_ros2_driver',
                                executable='ydlidar_ros2_driver_node',
                                name='ydlidar_ros2_driver_node',
                                output='screen',
                                emulate_tty=True,
                                parameters=[parameter_file],
                                )
     tf2_node = Node(package='tf2_ros',
                    executable='static_transform_publisher',
                    name='static_tf_pub_laser',
                    arguments=['0', '0', '0.02','0', '0', '0', '1','base_link','laser_frame'],
                    )

```

- find parameters:
```
browse https://github.com/YDLIDAR/ydlidar_ros2_driver/blob/master/details.md  
  also in src/ydlidar_ros2_ws/src/ydlidar_ros2_driver/details.md  


LIDAR: X4
Model: 6  (device_type?)
Baudrate: 128000
SampleRate(K): 5
Range(m): 0.12~10
Freq(Hz): 5~12(PWM)
Intensity(bit): false
SingleChannel: false
Voltage(V): 4.8~5.2
lidar_type: TYPE_TRIANGLE (1)
Reversion: false (CounterClockWise)
support_motor_dtr: true
```

- Configure:
```
cp src/ydlidar_ros2_ws/src/ydlidar_ros2_driver/params/ydlidar.yaml src/ydlidar_ros2_ws/src/ydlidar_ros2_driver/params/ydlidar.yaml.orig

nano src/ydlidar_ros2_ws/src/ydlidar_ros2_driver/params/ydlidar.yaml
ydlidar_ros2_driver_node:
  ros__parameters:
    port: /dev/ttyUSB0
    frame_id: laser_frame
    ignore_array: ""
    baudrate: 128000
    lidar_type: 1
    device_type: 6
    sample_rate: 5
    abnormal_check_count: 4
    resolution_fixed: true
    reversion: false
    inverted: true  
    auto_reconnect: true
    isSingleChannel: false
    intensity: false
    support_motor_dtr: true
    angle_max: 180.0
    angle_min: -180.0
    range_max: 10.0
    range_min: 0.12
    frequency: 5.0
    invalid_range_is_inf: false
```


- MODIFY SERIAL PORT NAME
```
chmod 777 src/ydlidar_ros2_ws/src/ydlidar_ros2_driver/startup/*
sudo sh src/ydlidar_ros2_ws/src/ydlidar_ros2_driver/startup/initenv.sh

Unplug the LIDAR and replug
```

- LAUNCH GoPiGo3 Node
```
./runit.sh
```

- LAUNCH LIDAR Node (add to runit.sh when runs good)
```
ros2 launch ydlidar_ros2_driver ydlidar_launch.py 

pi@ROSbot:~/rosbot-on-gopigo3$ ros2 launch ydlidar_ros2_driver ydlidar_launch.py 
[INFO] [launch]: All log files can be found below /home/pi/.ros/log/2021-08-04-10-38-45-809263-ROSbot-7654
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [ydlidar_ros2_driver_node-1]: process started with pid [7677]
[INFO] [static_transform_publisher-2]: process started with pid [7679]
[ydlidar_ros2_driver_node-1] [INFO] [1628087926.833046492] [ydlidar_ros2_driver_node]: [YDLIDAR INFO] Current ROS Driver Version: 1.0.1
[ydlidar_ros2_driver_node-1] 
[ydlidar_ros2_driver_node-1] YDLidar SDK initializing
[ydlidar_ros2_driver_node-1] YDLidar SDK has been initialized
[ydlidar_ros2_driver_node-1] [YDLIDAR]:SDK Version: 1.0.3
[static_transform_publisher-2] [INFO] [1628087926.878554381] [static_tf_pub_laser]: Spinning until killed publishing transform from 'base_link' to 'laser_frame'
[ydlidar_ros2_driver_node-1] LiDAR successfully connected
[ydlidar_ros2_driver_node-1] [YDLIDAR]:Lidar running correctly ! The health status: good
[ydlidar_ros2_driver_node-1] [YDLIDAR] Connection established in [/dev/ttyUSB0][128000]:
[ydlidar_ros2_driver_node-1] Firmware version: 1.10
[ydlidar_ros2_driver_node-1] Hardware version: 1
[ydlidar_ros2_driver_node-1] Model: X4
[ydlidar_ros2_driver_node-1] Serial: 2020062200002315
[ydlidar_ros2_driver_node-1] LiDAR init success!
[ydlidar_ros2_driver_node-1] [YDLIDAR3]:Fixed Size: 1020
[ydlidar_ros2_driver_node-1] [YDLIDAR3]:Sample Rate: 5K
[ydlidar_ros2_driver_node-1] [YDLIDAR INFO] Current Sampling Rate : 5K
[ydlidar_ros2_driver_node-1] [YDLIDAR INFO] Now YDLIDAR is scanning ......
^C[WARNING] [launch]: user interrupted with ctrl-c (SIGINT)
[ydlidar_ros2_driver_node-1] [INFO] [1628087941.729329370] [rclcpp]: signal_handler(signal_value=2)
[ydlidar_ros2_driver_node-1] [INFO] [1628087941.834658875] [ydlidar_ros2_driver_node]: [YDLIDAR INFO] Now YDLIDAR is stopping .......
[INFO] [static_transform_publisher-2]: process has finished cleanly [pid 7679]
[static_transform_publisher-2] [INFO] [1628087941.729325515] [rclcpp]: signal_handler(signal_value=2)
[ydlidar_ros2_driver_node-1] 281472982034848 thread has been canceled
[ydlidar_ros2_driver_node-1] [YDLIDAR INFO] Now YDLIDAR Scanning has stopped ......
[INFO] [ydlidar_ros2_driver_node-1]: process has finished cleanly [pid 7677]


```

- DESKTOP LAUNCH RVIZ2
```
cd ~/handsonros2
source install/setup.bash
./launch_rviz2_w_joint_and_robot_pubs.sh
```

- CONFIGURE RVIZ2 
```
Global Options
  Fixed Frame: laslerl_frame
  Color: 48; 48; 48
  Rate: 30
Grid
  Reference Frame: <Fixed Frame>
  Plane Count: 50
  Normal Count: 0
  Cell Size: 0.1
  Line Style: Lines
  Color: 160; 160; 164
  Alpha: 0.5
  Plane: XY
  Offset: 0; 0; 0
RobotModel
  Source: File
  File: ../src/chap4_rviz_basics/urdf/gpgMin.urdf
```

- ADD LASERSCAN DISPLAY
```
Add (Display) -> LaserScan
  Topic /scan
  Depth 5
  History: Keep Last
  Reliability System Default
  Durability: Vollatile
  Selectable: check
  Style: Flat Square
  Size(m): 0.02
  Alpha: 1
  Decay Time: 0
  Position Transformer: XYZ
  Color Transformer: FlatColor
  Color: 239; 41; 41
```

- SAVE RVIZ CONFIGURATION
```
File->save
File->save config as: (/home/ubuntu/.rviz2/) gpgMin_w_LIDAR.rviz
```
