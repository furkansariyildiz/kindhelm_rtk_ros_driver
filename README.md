# kindhelm_rtk_ros_driver
Kindhelm Tiny ROS2 Foxy, Galactic driver. 

<p align="left">
  <img src="documents/kindhelm-logo-header.png" style="width: 10%; height: 10%"/>
  <img src="documents/ros2.png.png" style="width: 10%; height: 10%"/>
</p>


### To build kindhelm ROS2 package

```bash
cd ~ros2_ws/src && git clone https://github.com/furkansariyildiz/kindhelm_rtk_ros_driver.git
colcon build --symlink-install --packages-select kindhelm_rtk_ros_package
```


### Running package via ros2 run command

```bash
cd ~ros2_ws && source install/setup.bash
ros2 run kindhelm_rtk_ros_package tiny 
```

### Running package via ros2 launch command

```bash
cd ~ros2_ws && source install/setup.bash
ros2 launch kindhelm_rtk_ros_package tiny.launch.py
```

### If you are using ros2 launch command, you should configure config.yaml 
```yaml
/kindhelm_tiny_node:
  ros__parameters:
    port_name: "/dev/ttyUSB0" # Port name of Kindhelm Tiny
    baudrate: 115200 # Baudrate of Kindhelm Tiny.
```

### Contact with me all around the web:
[![LinkedIn Badge](https://img.shields.io/badge/LinkedIn-Profile-informational?style=flat&logo=linkedin&logoColor=white&color=0D76A8)](https://www.linkedin.com/in/furkan-sariyildiz/)