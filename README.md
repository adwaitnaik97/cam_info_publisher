# Camera Info Publisher

Migration of [camera info publisher](https://github.com/aidrivers/cam_info_publisher) package to ROS2.

# Usage

### Step 1: Install ROS 2 (preferably Foxy or the version compatible with Ubuntu 20.04 LTS)

### Step 2: Clone the repository

```bash
git clone https://github.com/adwaitnaik97/cam_info_publisher.git
cd cam_info_publisher
```

### Step 3: Build and Source

```bash
rosdep install -i --from-path src --rosdistro foxy -y #All required rosdeps installed successfully
colcon build --symlink-install
. install/setup.bash
```

### Step 4: Execute the node (Open multiple terminal windows)

```bash
ros2 launch cam_info_publisher cam_info_publisher.launch.py #Window1

ros2 topic list #Window2 | Output: {'/camera_info', '/image_raw'}

ros2 topic echo /camera_info #Window3

ros2 run cam_info_publisher test_publisher /home/adwait/workspace/ros2_packages/cam_info_publisher/extras/TurtleBot4.jpg #Window4 replace the image path with your custom path
```

# Testing

This video demonstrates the camera parameters being published on the `/camera_info` topic.

[Watch the demonstration video](https://github.com/adwaitnaik97/cam_info_publisher/blob/master/extras/cam_info_publisher.mp4)