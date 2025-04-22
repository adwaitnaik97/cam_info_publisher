# Camera Info Publisher

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

ros2 topic echo /image_raw #Window4
```