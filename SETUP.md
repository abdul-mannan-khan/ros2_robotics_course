# Development Environment Setup

Complete setup guide for the ROS2 Robotics Course.

## 1. System Requirements

Before starting, ensure your system meets these requirements:

- **Operating System**: Ubuntu 24.04 LTS (Noble Numbat)
- **RAM**: 8GB minimum, 16GB recommended for simulation
- **Disk Space**: 20GB free space for ROS2, dependencies, and course materials
- **GPU**: Optional, but recommended for Week 10 (Computer Vision) exercises
- **Processor**: Multi-core processor recommended for Gazebo simulation

## 2. Install ROS2 Jazzy Jalisco

ROS2 Jazzy Jalisco is the latest LTS release compatible with Ubuntu 24.04. Follow these steps to install:

```bash
# Update package index
sudo apt update && sudo apt install -y software-properties-common

# Enable Ubuntu Universe repository
sudo add-apt-repository universe

# Add ROS2 GPG key
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add ROS2 repository to sources list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update package index and install ROS2
sudo apt update
sudo apt install -y ros-jazzy-desktop ros-jazzy-rosbag2 ros-jazzy-cv-bridge ros-jazzy-tf2-ros ros-jazzy-nav2-bringup

# Install additional development tools
sudo apt install -y python3-colcon-common-extensions python3-rosdep

# Initialize rosdep
sudo rosdep init
rosdep update
```

### Verify Installation

```bash
source /opt/ros/jazzy/setup.bash
ros2 --version
```

You should see output like: `ros2 cli version 0.XX.X`

## 3. Python Dependencies

Install required Python packages for the course exercises:

```bash
# Core dependencies
pip install numpy opencv-python matplotlib --break-system-packages

# Optional: Additional packages for advanced exercises
pip install scipy scikit-learn pandas --break-system-packages
```

Note: The `--break-system-packages` flag is required on Ubuntu 24.04 due to PEP 668 externally managed environments. Alternatively, you can use a virtual environment:

```bash
python3 -m venv ~/ros2_venv
source ~/ros2_venv/bin/activate
pip install numpy opencv-python matplotlib scipy scikit-learn pandas
```

## 4. Clone and Build the Course

### Clone Repository

```bash
# Create workspace
mkdir -p ~/auto_ws/src && cd ~/auto_ws/src

# Clone the course repository
git clone https://github.com/abdul-mannan-khan/robotics-from-zero-to-hired.git
```

Replace `YOUR_USERNAME` with the actual GitHub username or organization.

### Create Symlinks for Package Discovery

The course uses a nested package structure. Create symlinks so colcon can discover individual week packages:

```bash
cd ~/auto_ws

# Create symlinks for each week
for d in src/robotics-from-zero-to-hired/week_*/; do
    week=$(basename "$d")
    ln -sf "robotics-from-zero-to-hired/$week" "src/$week"
done
```

### Build the Workspace

```bash
cd ~/auto_ws

# Build all packages
colcon build --symlink-install

# Source the workspace
source install/setup.bash

# Add to bashrc for automatic sourcing in new terminals
echo "source ~/auto_ws/install/setup.bash" >> ~/.bashrc
```

The `--symlink-install` flag creates symbolic links instead of copying files, which speeds up rebuilds during development.

### Verify Build

```bash
# List available packages
ros2 pkg list | grep week

# You should see packages like:
# week_01_lidar_processing
# week_02_sensor_fusion
# week_03_transforms
# etc.
```

## 5. Generate Synthetic Bag Data

Many exercises use ROS2 bag files with synthetic sensor data. Generate these bags before running exercises:

```bash
# Generate bags for all weeks
for d in ~/auto_ws/src/robotics-from-zero-to-hired/week_*/w*_exercises/; do
    if [ -f "$d/generate_bag.py" ]; then
        echo "Generating bag for $(dirname $d)..."
        cd "$d" && python3 generate_bag.py && cd -
    fi
done
```

This will create bag files in each week's `bags/` directory. Generation may take a few minutes.

### Manual Generation for Specific Weeks

If you need to regenerate data for a specific week:

```bash
cd ~/auto_ws/src/robotics-from-zero-to-hired/week_01/w01_exercises
python3 generate_bag.py
```

## 6. Running Exercises

### Launch Demo Node

Each week includes a demo launch file that starts all nodes and visualization:

```bash
# Source workspace
source ~/auto_ws/install/setup.bash

# Run week 1 demo
ros2 launch week_01_lidar_processing demo.launch.py

# Run week 2 demo
ros2 launch week_02_sensor_fusion demo.launch.py
```

This will:
- Play back the synthetic bag file
- Launch exercise nodes
- Open RViz2 with pre-configured visualization

### Run Individual Exercise Nodes

To run a single exercise node without the full demo:

```bash
# Run a specific exercise node
ros2 run week_01_lidar_processing ex1_ground_removal

# In another terminal, play the bag file
ros2 bag play ~/auto_ws/src/robotics-from-zero-to-hired/week_01/w01_exercises/bags/lidar_data
```

### Visualize in RViz2

Launch RViz2 with a pre-configured layout:

```bash
rviz2 -d ~/auto_ws/src/robotics-from-zero-to-hired/week_01/w01_exercises/config/rviz_config.rviz
```

Or launch RViz2 separately and manually add displays:

```bash
rviz2
```

## 7. Troubleshooting

### RViz2 Snap Library Conflict

**Problem**: RViz2 crashes with library conflicts when installed via snap.

**Solution**: The demo launch files automatically filter snap paths from `LD_LIBRARY_PATH`. If you still encounter issues:

```bash
# Uninstall snap version
sudo snap remove rviz2

# Install from apt
sudo apt install ros-jazzy-rviz2
```

### rosbag2_py Import Error

**Problem**: `ModuleNotFoundError: No module named 'rosbag2_py'`

**Solution**: Ensure rosbag2 is installed:

```bash
sudo apt install ros-jazzy-rosbag2 ros-jazzy-rosbag2-py
source /opt/ros/jazzy/setup.bash
```

### colcon Can't Find Packages

**Problem**: `Package 'week_XX' not found` after building.

**Solution**: Ensure symlinks are created correctly:

```bash
cd ~/auto_ws
ls -la src/ | grep week

# You should see symlinks like:
# week_01 -> robotics-from-zero-to-hired/week_01
# week_02 -> robotics-from-zero-to-hired/week_02

# If missing, recreate symlinks
for d in src/robotics-from-zero-to-hired/week_*/; do
    week=$(basename "$d")
    ln -sf "robotics-from-zero-to-hired/$week" "src/$week"
done

# Rebuild
colcon build --symlink-install
```

### Python Module Not Found in Nodes

**Problem**: Exercise nodes can't find custom Python modules.

**Solution**: Ensure you sourced the workspace and used `--symlink-install`:

```bash
source ~/auto_ws/install/setup.bash
cd ~/auto_ws
colcon build --symlink-install
```

### Gazebo Simulation Runs Slowly

**Problem**: Gazebo simulation is laggy or runs in slow motion.

**Solution**:
- Close unnecessary applications to free up RAM
- Reduce simulation quality in Gazebo settings
- Check if GPU drivers are properly installed:

```bash
# For NVIDIA GPUs
nvidia-smi

# Install NVIDIA drivers if needed
sudo ubuntu-drivers autoinstall
```

### Transform Lookup Failures

**Problem**: `Could not transform point cloud from 'sensor_frame' to 'base_link'`

**Solution**:
- Ensure the transform publisher node is running
- Check that frame IDs match in your code and URDF
- Use `tf2_tools` to debug:

```bash
# View transform tree
ros2 run tf2_tools view_frames

# Echo specific transform
ros2 run tf2_ros tf2_echo base_link sensor_frame
```

### Permission Denied When Running Nodes

**Problem**: `Permission denied` when executing Python nodes.

**Solution**: Make Python files executable:

```bash
chmod +x ~/auto_ws/src/robotics-from-zero-to-hired/week_*/w*_exercises/scripts/*.py
```

### Build Errors After Pulling Updates

**Problem**: Build fails after pulling new changes from git.

**Solution**: Clean and rebuild:

```bash
cd ~/auto_ws
rm -rf build install log
colcon build --symlink-install
```

## Additional Resources

- **ROS2 Documentation**: https://docs.ros.org/en/jazzy/
- **ROS2 Tutorials**: https://docs.ros.org/en/jazzy/Tutorials.html
- **Gazebo Documentation**: https://gazebosim.org/docs
- **Course Issues**: Report problems on the GitHub repository

## Next Steps

After completing the setup:

1. Verify installation by running Week 1 exercises
2. Read through `INTERVIEW_PREP.md` to understand learning objectives
3. Join the course community on Discord/Slack (if available)
4. Start with Week 1: LiDAR Processing and Ground Removal

Happy learning!
