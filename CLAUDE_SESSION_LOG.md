# Claude Session Log — Robotics Course Development

## Project Overview
A 12-week open-source Robotics & Drone Engineering course built on ROS2 Jazzy Jalisco (Ubuntu 24.04). Designed for interview preparation targeting Amazon Robotics, Google/Waymo, Boston Dynamics, Skydio, Tesla, DJI.

**GitHub**: https://github.com/abdul-mannan-khan/robotics-drone-interview-based-course
**Author**: Dr. Abdul Manan Khan (abdul.mannan.617@gmail.com)
**Workspace**: /home/it-services/auto_ws/
**Source**: /home/it-services/auto_ws/src/ros2_robotics_course/

---

## What Was Built (Complete)

### 1. HTML Lectures (12 weeks)
- Original lectures in `week_XX/lecture/` (contain UWL branding — gitignored, NOT on GitHub)
- Public versions in `week_XX/lecture_public/` (UWL branding removed — on GitHub)
- Script to regenerate: `/tmp/claude-1000/-home-it-services-auto-ws-src/4db894d5-7232-4bfb-abba-8d640dff42fb/scratchpad/clean_lectures.py`
- All MathJax-enabled with proper math symbol definitions

### 2. Theory Exercises (12 weeks)
- `week_XX/lab_exercises/` — Exercise stubs with TODOs
- `week_XX/lab_solutions/` — Complete solutions
- Include .npy data files for weeks 1-5, 8

### 3. ROS2 Applied Exercises (12 weeks, 3 per week = 36 total)
- `week_XX/wXX_exercises/` — Exercise node stubs with TODOs
- `week_XX/wXX_solutions/` — Complete solution nodes
- Each week has: exercise1_node.py, exercise2_node.py, exercise3_node.py + matching solutions

### 4. ROS2 Infrastructure Per Week
- `wXX_exercises/launch/` — exercise1-3.launch.py + demo.launch.py
- `wXX_exercises/config/params.yaml` — ROS2 parameters
- `wXX_exercises/config/rviz_config.rviz` — RViz2 visualization config
- `wXX_exercises/generate_bag.py` — Generates synthetic ROS2 bags using rosbag2_py
- `wXX_exercises/download_data.sh` — Downloads public datasets (optional)
- `wXX_exercises/bag_data/` — Generated bag storage (gitignored)

### 5. ROS2 Packages (13 total)
| Package | Dir |
|---------|-----|
| ros2_robotics_course (metapackage) | . |
| week_01_lidar_processing | week_01/ |
| week_02_sensor_fusion | week_02/ |
| week_03_ros2_fundamentals | week_03/ |
| week_04_slam_2d | week_04/ |
| week_05_nav2 | week_05/ |
| week_06_quadrotor_dynamics | week_06/ |
| week_07_px4_integration | week_07/ |
| week_08_3d_path_planning | week_08/ |
| week_09_ego_planner | week_09/ |
| week_10_computer_vision | week_10/ |
| week_11_control_systems | week_11/ |
| week_12_capstone | week_12/ |

### 6. GitHub Publication Files
- `README.md` — Full curriculum, quick start, video resources, interview prep, learning approach
- `INTERVIEW_PREP.md` — Company-specific interview guides (Amazon, Google, Boston Dynamics, Skydio, Tesla, DJI)
- `SETUP.md` — ROS2 Jazzy installation + workspace setup + troubleshooting
- `LICENSE` — Apache 2.0
- `.gitignore` — Python, ROS2, bags, IDE, UWL lectures

---

## Directory Structure
```
ros2_robotics_course/
├── README.md, INTERVIEW_PREP.md, SETUP.md, LICENSE, .gitignore
├── CLAUDE_SESSION_LOG.md          # This file
├── package.xml, setup.py, setup.cfg  # Metapackage
├── week_01/ through week_12/
│   ├── package.xml, setup.py, setup.cfg  # Per-week ROS2 package
│   ├── resource/week_XX_NAME             # ament resource marker
│   ├── lecture/                    # Original UWL lectures (gitignored)
│   ├── lecture_public/             # Cleaned public lectures
│   ├── lab_exercises/              # Theory exercises
│   ├── lab_solutions/              # Theory solutions
│   ├── wXX_exercises/              # ROS2 exercise stubs
│   │   ├── __init__.py
│   │   ├── exercise1_node.py, exercise2_node.py, exercise3_node.py
│   │   ├── generate_bag.py
│   │   ├── download_data.sh
│   │   ├── README.md
│   │   ├── launch/ (exercise1-3.launch.py, demo.launch.py)
│   │   ├── config/ (params.yaml, rviz_config.rviz)
│   │   └── bag_data/ (synthetic bags, gitignored)
│   └── wXX_solutions/              # Complete ROS2 solutions
│       ├── __init__.py
│       └── solution1_node.py, solution2_node.py, solution3_node.py
└── week_09/ego-planner-swarm/      # External repo (gitignored)
```

---

## Key Technical Details

### Build System
- ament_python packages, built with `colcon build --symlink-install`
- **Symlinks required**: Colcon can't discover nested packages, so symlinks must exist:
  ```bash
  for d in src/ros2_robotics_course/week_*/; do
      week=$(basename "$d")
      ln -sf "ros2_robotics_course/$week" "src/$week"
  done
  ```
- Python modules named `wXX_exercises` / `wXX_solutions` (unique per package to avoid collisions)

### Bag Generation
- Uses native `rclpy` + `rosbag2_py` (NOT the `rosbags` pip package — its API changed)
- ROS2 Jazzy requires `id=0` in `rosbag2_py.TopicMetadata()`
- Uses `rclpy.serialization.serialize_message()` for CDR serialization
- Bag names: synthetic_lidar, synthetic_euroc, synthetic_turtlebot, synthetic_2d_slam, synthetic_nav2, synthetic_quadrotor, synthetic_px4, synthetic_3d_env, synthetic_ego_planner, synthetic_vision, synthetic_control, synthetic_capstone

### Demo Launch Files
- Use `os.path.realpath(__file__)` (not abspath) to resolve through symlinks to source dir
- Filter snap paths from LD_LIBRARY_PATH to prevent RViz2 crash:
  ```python
  clean_ld = ':'.join(p for p in ld_path.split(':') if 'snap' not in p)
  ```
- Pattern: bag play (rate 2.0, loop) → 2s delay → 3 solution nodes → 3s delay → RViz2

### Known System Issues (Not Code Bugs)
- **CycloneDDS domain exhaustion**: `Failed to find a free participant index for domain 0` — happens when starting/stopping many ROS2 nodes rapidly. Fix: wait between runs, or reboot
- **RViz2 snap conflict**: `libpthread.so.0: undefined symbol: __libc_pthread_init` — snap's libraries conflict with system. Fix: filter snap from LD_LIBRARY_PATH (done in launch files)
- **pip on Ubuntu 24.04**: Requires `--break-system-packages` flag for system Python

---

## Bugs Found and Fixed

1. **Wrong directory names**: Agents created `week1/` instead of `week_01/` → moved to correct dirs
2. **Module name collisions**: All packages shared `ros2_exercises` module → renamed to `wXX_exercises`
3. **resource file was directory**: week_08 resource was a dir → recreated as file
4. **rosbags API change**: `serialize_cdr` unavailable → rewrote to use native rosbag2_py
5. **TopicMetadata missing id**: Jazzy requires `id=0` parameter
6. **PointCloud2 reshape**: `reshape(-1, 3)` fails with point_step=16 → `reshape(-1, msg.point_step // 4)[:, :3]`
7. **Launch file path resolution**: `os.path.abspath` doesn't follow symlinks → `os.path.realpath`
8. **Week 10 ValueError**: `disparity[valid].min()` on empty array → added `if valid.any()` guard

---

## Testing Results
All 36 solution nodes (12 weeks x 3) tested and confirmed working:
- All start without Python errors
- Data flows verified: bag → node → published topics
- Week 1 verified output: "Scan N: pts=100929, min=3.51, max=73.55, mean=10.58"
- Week 8 verified output: "Published 8318 voxels"

---

## Git History
```
d69ce1f Initial commit: 12-week Robotics & Drone Engineering open course (694 files)
fb3a129 Update GitHub URLs to actual repository path
c7e5ba7 Remove original UWL-branded lectures, keep only public versions
479a403 Rename repo to robotics-from-zero-to-hired
83cba53 Rename repo to robotics-drone-interview-based-course
cbb051a Fix demo launch path resolution and week 10 zero-size array bug
```

---

## How to Rebuild From Scratch
```bash
# 1. Source ROS2
source /opt/ros/jazzy/setup.bash

# 2. Create symlinks (if not present)
cd ~/auto_ws/src
for d in ros2_robotics_course/week_*/; do
    week=$(basename "$d")
    ln -sf "ros2_robotics_course/$week" "$week" 2>/dev/null
done

# 3. Build
cd ~/auto_ws
colcon build --symlink-install
source install/setup.bash

# 4. Generate bags
for d in ~/auto_ws/src/ros2_robotics_course/week_*/w*_exercises/; do
    [ -f "$d/generate_bag.py" ] && (cd "$d" && python3 generate_bag.py)
done

# 5. Run any week's demo
ros2 launch week_01_lidar_processing demo.launch.py
```

---

## UWL Branding Removal
- Original lectures in `week_XX/lecture/` contain University of West London (UWL) branding and module code TC70045E
- These are gitignored and NOT on GitHub
- Public versions in `week_XX/lecture_public/` have all UWL references replaced with "Open Robotics Course" / "Robotics & Drone Engineering"
- Script to regenerate public versions: run `clean_lectures.py` from scratchpad
- Verified: zero UWL references in any git-tracked file
