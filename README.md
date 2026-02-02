# Robotics & Drone Engineering — Open Course

**A practical 12-week ROS2 course focused on real-world robotics skills and interview preparation for companies like Amazon Robotics, Google/Waymo, Boston Dynamics, Skydio, and DJI**

**Author:** Dr. Abdul Manan Khan
**ROS2 Distribution:** Jazzy Jalisco (Ubuntu 24.04)

---

## Course Overview

This hands-on ROS2 course covers essential robotics topics including LiDAR point cloud processing, sensor fusion with Extended Kalman Filters, SLAM (Simultaneous Localization and Mapping), autonomous navigation, quadrotor dynamics, 3D path planning, computer vision, and advanced control systems. Each week features HTML lecture slides, theory exercises, and applied ROS2 node implementations with synthetic bag data for realistic testing. The curriculum is designed to prepare students for robotics engineering roles at leading companies in autonomous vehicles, warehouse automation, and drone technology.

---

## Quick Start

```bash
# Prerequisites: ROS2 Jazzy on Ubuntu 24.04
sudo apt install ros-jazzy-desktop
source /opt/ros/jazzy/setup.bash

# Clone and build
mkdir -p ~/auto_ws/src && cd ~/auto_ws/src
git clone https://github.com/abdul-mannan-khan/ros2_robotics_course.git
cd ~/auto_ws
colcon build --symlink-install
source install/setup.bash

# Run a demo (e.g., Week 1 LiDAR Processing)
cd src/ros2_robotics_course/week_01/w01_exercises
python3 generate_bag.py
ros2 launch week_01_lidar_processing demo.launch.py
```

---

## 12-Week Curriculum

| Week | Topic | Key Concepts | Industry Relevance |
|------|-------|--------------|-------------------|
| 1 | LiDAR Point Cloud Processing | Voxel grid filtering, ground segmentation, Euclidean clustering | Waymo, Luminar, Velodyne, Ouster |
| 2 | Sensor Fusion & EKF | Extended Kalman Filter, IMU+GPS fusion, covariance estimation | Waymo, Cruise, Nuro, Aurora |
| 3 | ROS2 Fundamentals | Topics, services, actions, lifecycle nodes, QoS policies | All robotics companies |
| 4 | 2D SLAM | Occupancy grid mapping, scan matching, loop closure detection | Amazon Robotics, iRobot, Locus Robotics |
| 5 | Nav2 Navigation | Costmaps, global/local planners, behavior trees, recovery behaviors | Amazon warehouse robots, Locus Robotics, Fetch Robotics |
| 6 | Quadrotor Dynamics | Newton-Euler equations, thrust model, attitude control, ESCs | DJI, Skydio, Zipline, Wing |
| 7 | PX4/ArduPilot Integration | MAVLink protocol, offboard control, SITL simulation, mission planning | Auterion, ArduPilot ecosystem, Dronecode |
| 8 | 3D Path Planning | RRT*, A* in 3D space, octree occupancy grids | Amazon Prime Air, Skydio, Wing, Zipline |
| 9 | EGO-Planner | B-spline trajectory optimization, ESDF maps, local replanning | Autonomous drone research, Skydio |
| 10 | Computer Vision | YOLO object detection, stereo depth estimation, visual odometry | Tesla, Waymo, Amazon Go, Cruise |
| 11 | Control Systems | PID tuning, LQR optimal control, Model Predictive Control | Boston Dynamics, Agility Robotics, ANYbotics |
| 12 | Capstone Integration | Full autonomy pipeline, multi-sensor fusion, system architecture | Interview project showcase for all companies |

---

## Video Lecture Resources

| Week | Topic | Recommended Videos |
|------|-------|-------------------|
| 1 | LiDAR Point Cloud Processing | [Cyrill Stachniss - Photogrammetry and Robotics](https://www.youtube.com/c/CyrillStachniss), [Point Cloud Processing Tutorial](https://www.youtube.com/results?search_query=lidar+point+cloud+processing+tutorial) |
| 2 | Sensor Fusion & EKF | [Michel van Biezen - Kalman Filter](https://www.youtube.com/results?search_query=michel+van+biezen+kalman+filter), [MATLAB Tech Talks - Understanding Kalman Filters](https://www.youtube.com/results?search_query=matlab+understanding+kalman+filters) |
| 3 | ROS2 Fundamentals | [Articulated Robotics - ROS2 Tutorials](https://www.youtube.com/@ArticulatedRobotics), [ROS2 Official Documentation](https://docs.ros.org/en/jazzy/Tutorials.html) |
| 4 | 2D SLAM | [Cyrill Stachniss - SLAM Course](https://www.youtube.com/results?search_query=cyrill+stachniss+slam), [SLAM Lectures - Mobile Sensing and Robotics](https://www.youtube.com/results?search_query=slam+lecture+mobile+robotics) |
| 5 | Nav2 Navigation | [Nav2 Official Tutorials](https://www.youtube.com/results?search_query=nav2+ros2+tutorial), [ROS Navigation Tuning Guide](https://www.youtube.com/results?search_query=ros+navigation+stack+tutorial) |
| 6 | Quadrotor Dynamics | [ETH Zurich - Flying Machine Arena](https://www.youtube.com/results?search_query=eth+zurich+flying+machine+arena), [Drone Dynamics and Control](https://www.youtube.com/results?search_query=quadrotor+dynamics+control+lecture) |
| 7 | PX4/ArduPilot Integration | [PX4 Developer Summit Talks](https://www.youtube.com/results?search_query=px4+developer+summit), [ArduPilot Developer Videos](https://www.youtube.com/results?search_query=ardupilot+mavlink+tutorial) |
| 8 | 3D Path Planning | [Sebastian Thrun - AI for Robotics](https://www.youtube.com/results?search_query=sebastian+thrun+path+planning), [RRT* Algorithm Explanation](https://www.youtube.com/results?search_query=rrt+star+algorithm+explained) |
| 9 | EGO-Planner | [Trajectory Optimization Lectures](https://www.youtube.com/results?search_query=trajectory+optimization+robotics), [B-spline Path Planning](https://www.youtube.com/results?search_query=bspline+trajectory+optimization) |
| 10 | Computer Vision | [Andrej Karpathy - Neural Networks](https://www.youtube.com/@AndrejKarpathy), [Stanford CS231n](https://www.youtube.com/results?search_query=stanford+cs231n+convolutional+neural+networks) |
| 11 | Control Systems | [Brian Douglas - Control Systems Lectures](https://www.youtube.com/@BrianBDouglas), [Steve Brunton - Control Bootcamp](https://www.youtube.com/@Eigensteve) |
| 12 | Capstone Integration | Review previous weeks, focus on system integration and portfolio building |

---

## Interview Preparation

This course is designed with robotics industry interviews in mind. Each week's exercises build practical skills that directly translate to interview success at top robotics companies.

### Common Interview Topics by Company

- **Amazon Robotics**: Warehouse navigation algorithms, 2D SLAM, sensor fusion for localization, fleet management system design, path planning in dynamic environments
- **Google/Waymo**: LiDAR and camera perception pipelines, prediction and planning under uncertainty, sensor calibration, behavior planning, simulation frameworks
- **Boston Dynamics**: Robot dynamics and kinematics, advanced control theory (LQR, MPC), state estimation, real-time systems, actuator control
- **Skydio/DJI**: Visual SLAM, 3D path planning with obstacle avoidance, PX4/ArduPilot integration, computer vision for tracking, battery optimization
- **Tesla Autopilot**: Multi-camera fusion, deep learning for perception, sensor fusion (vision + radar), neural network deployment, map building

### Interview Tips

- **Build a portfolio**: Complete each week's exercises and showcase them on GitHub with clear README files and demo videos
- **Whiteboard readiness**: Practice implementing RRT*, EKF prediction/update steps, and PID controllers from scratch on paper
- **Master ROS2 concepts**: Know the differences between topics, services, and actions; understand QoS policies; explain lifecycle node states
- **System design practice**: Be prepared for questions like "Design a warehouse robot navigation system" or "How would you implement collision avoidance for a drone swarm?"
- **Know your fundamentals**: Linear algebra (transforms, rotation matrices), probability (Bayes rule, Gaussian distributions), algorithms (graph search, optimization)
- **Real-world trade-offs**: Be ready to discuss computational constraints, sensor limitations, safety requirements, and failure modes

---

## How to Learn

**Recommended weekly approach:**

1. **Watch video lectures first** - Get conceptual understanding from the recommended YouTube resources
2. **Read the HTML lecture slides** - Open `lecture_public/index.html` in your browser for detailed theory
3. **Complete theory exercises** - Work through problems in `lab_exercises/` to solidify mathematical concepts
4. **Implement ROS2 nodes** - Fill in TODOs in `wXX_exercises/` to build practical coding skills
5. **Compare with solutions** - Check your work against `lab_solutions/` and `wXX_solutions/`
6. **Visualize everything in RViz2** - See your algorithms working in real-time with point clouds, transforms, and paths
7. **Modify and experiment** - Change parameters in `config/`, try edge cases, break things and fix them

**Time commitment:** Expect 8-12 hours per week for video lectures, theory exercises, and coding implementations.

---

## Repository Structure

```
ros2_robotics_course/
├── README.md                  # This file
├── INTERVIEW_PREP.md          # Detailed interview guide
├── SETUP.md                   # Installation and environment setup
├── LICENSE                    # Apache 2.0 license
├── .gitignore                 # Git ignore rules
├── week_01/                   # LiDAR Point Cloud Processing
│   ├── lecture/               # Original HTML lectures
│   ├── lecture_public/        # Cleaned public versions
│   ├── lab_exercises/         # Theory exercises (PDF/Jupyter)
│   ├── lab_solutions/         # Theory solutions
│   ├── w01_exercises/         # ROS2 applied exercises (starter code)
│   │   ├── launch/            # Launch files
│   │   ├── config/            # Parameter YAML files
│   │   ├── bag_data/          # Synthetic rosbag2 data
│   │   ├── package.xml
│   │   ├── setup.py
│   │   └── *.py               # ROS2 Python nodes with TODOs
│   └── w01_solutions/         # Complete ROS2 solutions
├── week_02/ through week_12/  # Same structure for all weeks
│   └── ...
```

---

## ROS2 Compatibility

**Tested Configuration:**
- **ROS2 Distribution:** Jazzy Jalisco (May 2024 release)
- **Operating System:** Ubuntu 24.04 LTS (Noble Numbat)
- **Python Version:** 3.12
- **Build System:** colcon

**Key Dependencies:**
```bash
# Core ROS2 packages
ros-jazzy-desktop
ros-jazzy-navigation2
ros-jazzy-nav2-bringup
ros-jazzy-slam-toolbox
ros-jazzy-gazebo-ros-pkgs
ros-jazzy-robot-localization
ros-jazzy-tf2-tools
ros-jazzy-rviz2

# Python dependencies
pip3 install numpy scipy matplotlib opencv-python opencv-contrib-python
pip3 install scikit-learn open3d pandas shapely
```

**Additional Tools:**
- **Simulation:** Gazebo Classic or Gazebo Sim (Harmonic)
- **Visualization:** RViz2, Foxglove Studio
- **Bag Recording:** rosbag2 with sqlite3 storage
- **PX4 SITL:** For drone simulation weeks (7-9)

---

## Contributing

Contributions are welcome! If you find bugs, have suggestions for improvements, or want to add supplementary materials:

1. Fork this repository
2. Create a feature branch (`git checkout -b feature/your-feature`)
3. Commit your changes (`git commit -m 'Add some feature'`)
4. Push to the branch (`git push origin feature/your-feature`)
5. Open a Pull Request

**Guidelines:**
- Follow PEP 8 style for Python code
- Add docstrings to all functions and classes
- Include launch files and config files for new nodes
- Update the relevant week's README with your changes
- Test your code with ROS2 Jazzy before submitting

---

## License

This project is licensed under the **Apache License 2.0** - see the [LICENSE](LICENSE) file for details.

Copyright 2025 Dr. Abdul Manan Khan

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

---

## Contact & Support

**Course Author:** Dr. Abdul Manan Khan
**Repository:** [https://github.com/abdul-mannan-khan/ros2_robotics_course](https://github.com/abdul-mannan-khan/ros2_robotics_course)

For questions, issues, or discussions:
- Open an issue on GitHub
- Check existing issues for common problems
- Review the SETUP.md for installation troubleshooting

**Star this repository** if you find it helpful, and share it with others learning robotics!

---

**Happy Learning and Building!**
