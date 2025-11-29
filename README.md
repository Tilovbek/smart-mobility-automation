# Smart Mobility Automation for TurtleBot3

An integrated automation framework designed for TurtleBot3 robots running on ROS 2 Foxy (Ubuntu 20.04 LTS). This toolkit delivers autonomous path planning, real-time YOLOv8-based vision processing, continuous system health monitoring, and color-based object tracking. All components are structured as independent ROS 2 nodes, accessible through both command-line tools and launch configurations.

## Core Capabilities

- **Autonomous Navigation** – Full SLAM capabilities and waypoint-based navigation powered by the Nav2 framework
- **Vision Processing** – Live YOLOv8 object recognition integrated with ROS 2 messaging infrastructure
- **System Health Tracking** – Continuous monitoring of battery levels, diagnostics, and overall robot health with notification system
- **Color-Based Tracking** – Visual servoing system for detecting and following colored objects
- **Hardware-Free Testing** – Bash-based simulation scripts for validating functionality without physical robots

## Getting Started

### Step 1: Repository Setup and Environment Configuration

```bash
# Clone this repository
git clone https://github.com/Tilovbek/smart-mobility-automation
cd smart-mobility-automation

# Set up conda environment using Python 3.8 (ROS Foxy requirement)
conda create -n ros-foxy python=3.8 -y
conda activate ros-foxy

# Install required Python packages
pip install -r requirements.txt
```

### Step 2: ROS Foxy System Dependencies

```bash
# Refresh package index
sudo apt update

# Install ROS Foxy base distribution and TurtleBot3 ecosystem
sudo apt install -y ros-foxy-desktop ros-foxy-turtlebot3* ros-foxy-nav2* ros-foxy-slam-toolbox ros-foxy-vision-msgs

# Add build infrastructure
sudo apt install -y python3-colcon-common-extensions python3-rosdep
```

### Step 3: Compilation

```bash
# Set up rosdep
sudo rosdep init
rosdep update

# Compile workspace with symbolic links
colcon build --symlink-install
source install/setup.bash
```

### Step 4: Execution

#### Complete Autonomous System (Hardware Required)
```bash
# Start full navigation suite including SLAM
ros2 launch turtlebot3_automation navigation_only.launch.py
```

#### Isolated Modules
```bash
# Health monitoring system
ros2 launch turtlebot3_automation maintenance.launch.py

# Vision detection system
ros2 launch turtlebot3_automation object_detection.launch.py

# Color tracking system
ros2 launch turtlebot3_automation color_follow.launch.py
```

## Project Organization

```
src/turtlebot3_automation/
├── turtlebot3_automation/        # Core Python modules
│   ├── cli.py                     # Terminal interface
│   ├── maintenance/               # Health monitoring subsystem
│   ├── navigation/                # Path planning subsystem
│   ├── perception/                # Vision processing pipeline
│   ├── custom_features/           # Color-based tracking module
│   ├── setup_automation/          # Setup utilities
│   └── utils/                     # Common helper functions
├── config/                        # Parameter configuration files
├── launch/                        # Launch configurations
│   ├── navigation_only.launch.py  # Autonomous navigation launcher
│   ├── maintenance.launch.py      # Health monitoring launcher
│   ├── object_detection.launch.py # Vision system launcher
│   └── color_follow.launch.py     # Color tracking launcher
├── tests/                         # Test suites
└── package.xml / setup.py         # Package metadata
scripts/                           # Validation scripts
├── test_maintenance.sh            # Health system validation
├── test_detection.sh              # Vision system validation
└── test_color_follow.sh           # Color tracking validation
requirements.txt                   # Python package list
yolov8n.pt                        # Pre-trained neural network weights
```

## System Requirements

- **Operating System**: Ubuntu 20.04 LTS (Focal Fossa)
- **Python Version**: 3.8.x (managed via conda)
- **ROS Distribution**: ROS 2 Foxy (apt installation)
- **Robot Platform**: TurtleBot3 (physical hardware or Gazebo simulation)
- **GPU**: Optional (CPU inference supported for YOLOv8)

## Operation Guide

### Simulation Mode (No Hardware Required)

The project includes three validation scripts for testing subsystems without physical robots:

#### Health Monitoring Validation
```bash
# Generate synthetic battery telemetry
./scripts/test_maintenance.sh
```
Simulates voltage decline from 12.4V to 10.5V, activating health alerts.

#### Vision System Validation
```bash
# Generate synthetic detection events
./scripts/test_detection.sh
```
Emits randomized object class labels (pedestrians, vehicles, animals, etc.) for detection pipeline testing.

#### Color Tracking Validation
```bash
# Provide testing instructions and guidance
./scripts/test_color_follow.sh
```
Provides guidance on testing color tracking with physical objects or simulation.

**Testing Workflow:**
1. Terminal session 1: Start the subsystem under test
   ```bash
   ros2 launch turtlebot3_automation maintenance.launch.py
   ```
2. Terminal session 2: Execute the validation script
   ```bash
   ./scripts/test_maintenance.sh
   ```
3. Monitor output in terminal session 1

### Launch Configurations

- **Integrated Navigation Suite** (Mapping + Autonomy):
  ```bash
  ros2 launch turtlebot3_automation navigation_only.launch.py
  ```
  Activates Nav2 with SLAM Toolbox for simultaneous localization, mapping, and autonomous path execution.

- **Health Monitoring Service**:
  ```bash
  ros2 launch turtlebot3_automation maintenance.launch.py
  ```
  Tracks battery status, diagnostic messages, and system vitals.

- **Vision Processing Service**:
  ```bash
  ros2 launch turtlebot3_automation object_detection.launch.py
  ```
  Executes YOLOv8 inference on camera feed.

- **Color Tracking Service**:
  ```bash
  ros2 launch turtlebot3_automation color_follow.launch.py
  ```
  Activates colored object detection and visual servoing.

### Direct Node Execution

```bash
# Launch health monitor
ros2 run turtlebot3_automation maintenance_monitor

# Launch vision detector
ros2 run turtlebot3_automation yolo_detector

# Launch color tracker
ros2 run turtlebot3_automation color_follow

# Launch path planner
ros2 run turtlebot3_automation navigation_manager
```

## Configuration Details

### Health Monitoring Subsystem
- **Parameter File**: `src/turtlebot3_automation/config/maintenance.yaml`
- **Communication**:
  - Notification channel: `turtlebot3/alerts`
  - Status reports: 30-second intervals covering battery, diagnostics, and system state
  - Subscribed topics: `/battery_state`, `/diagnostics`, `/cmd_vel`

### Vision Processing Subsystem
- **Parameter File**: `src/turtlebot3_automation/config/object_detection.yaml`
- **Neural Network**: YOLOv8 nano variant (`yolov8n.pt`) with automatic download on initialization
- **Communication**:
  - Detection results: `turtlebot3/perception/detections` (`Detection2DArray`)
  - Visualization data: `turtlebot3/perception/markers` (`MarkerArray`)
  - Text output: `turtlebot3/perception/labels` (`String`)

### Autonomous Navigation Subsystem
- **Parameter File**: `src/turtlebot3_automation/config/navigation_params.yaml`
- **Functionality**: Simultaneous mapping, goal-based autonomy, collision avoidance
- **Architecture**: Nav2 framework with behavior trees, fallback strategies, and lifecycle node management

## Validation and Testing

### Simulation Scripts (Hardware Independent)

Validate subsystems using included bash scripts that inject synthetic data:

```bash
# Health monitoring validation
./scripts/test_maintenance.sh

# Vision system validation
./scripts/test_detection.sh

# Color tracking validation
./scripts/test_color_follow.sh
```

These scripts publish mock data to designated ROS 2 communication channels.

### Topic Inspection
```bash
# Inspect battery telemetry
ros2 topic echo /battery_state

# Inspect vision detections
ros2 topic echo turtlebot3/perception/labels

# Inspect health notifications
ros2 topic echo turtlebot3/alerts

# Inspect motion commands
ros2 topic echo /cmd_vel

# Measure publication frequencies
ros2 topic hz /battery_state
ros2 topic hz turtlebot3/perception/detections
```

### RViz Visualization
```bash
# Terminal 1: Start vision subsystem
ros2 launch turtlebot3_automation object_detection.launch.py

# Terminal 2: Open visualization tool
rviz2
```
RViz configuration:
1. Configure Fixed Frame as `base_link`
2. Add visualization: By topic → `/turtlebot3/perception/markers` → MarkerArray

### Automated Test Suite
Execute parameter validation tests:
```bash
colcon test --packages-select turtlebot3_automation
```

## Problem Resolution

### Frequent Issues

1. **ROS Distribution Mismatch**: Confirm ROS 2 Foxy installation (not Rolling or Galactic) paired with Python 3.8
2. **Environment Activation**: The `ros-foxy` conda environment must be active for all operations
3. **Compilation Problems**: Re-run `colcon build --symlink-install` following parameter modifications
4. **Navigation Dependencies**: The `ros-foxy-nav2-simple-commander` package requires system-level installation

### Script Execution Problems

**Permission denied errors:**
```bash
chmod +x scripts/*.sh
```

**Missing topic data:**
- Confirm node is active: `ros2 node list`
- Verify topic registration: `ros2 topic list`
- Inspect topic content: `ros2 topic echo /battery_state`

**Camera feed unavailable:**
- Vision and color tracking subsystems require camera publication
- Verify camera activity: `ros2 topic hz /camera/color/image_raw`
- Note: Simulation mode operates without camera topics

### Environment Verification

```bash
# Validate conda setup
conda activate ros-foxy
python --version  # Expected: 3.8.x

# Validate ROS installation
source /opt/ros/foxy/setup.bash
ros2 --version

# Validate package installation
source install/setup.bash
ros2 pkg list | grep turtlebot3_automation

# Enumerate available executables
ros2 pkg executables turtlebot3_automation
```

## Usage Examples

### Example 1: Health System Validation
```bash
# Session 1: Initialize health monitoring
ros2 launch turtlebot3_automation maintenance.launch.py

# Session 2: Execute battery simulation
./scripts/test_maintenance.sh
```

### Example 2: Vision System Validation
```bash
# Session 1: Initialize vision processing
ros2 launch turtlebot3_automation object_detection.launch.py

# Session 2: Inject detection events
./scripts/test_detection.sh
```

### Example 3: Multi-Channel Monitoring
```bash
# Session 1: Initialize health subsystem
ros2 launch turtlebot3_automation maintenance.launch.py

# Session 2: Track battery telemetry
ros2 topic echo /battery_state

# Session 3: Track health notifications
ros2 topic echo turtlebot3/alerts

# Session 4: Execute simulation
./scripts/test_maintenance.sh
```

## Communication Interfaces

### Output Channels

| Channel | Message Format | Content |
|---------|---------------|---------|
| `/battery_state` | `sensor_msgs/BatteryState` | Power system voltage and charge level |
| `/cmd_vel` | `geometry_msgs/Twist` | Motion control instructions |
| `turtlebot3/alerts` | `std_msgs/String` | System health notifications |
| `turtlebot3/perception/detections` | `vision_msgs/Detection2DArray` | Vision detection data |
| `turtlebot3/perception/labels` | `std_msgs/String` | Object class identifiers |
| `turtlebot3/perception/markers` | `visualization_msgs/MarkerArray` | Visualization elements for RViz |

### Input Channels

| Subsystem | Monitored Channels | Function |
|-----------|-------------------|----------|
| Health Monitor | `/battery_state`, `/diagnostics`, `/cmd_vel` | Tracks system vitals |
| Vision Processor | `/camera/color/image_raw` | Receives visual input |
| Color Tracker | `/camera/color/image_raw` | Receives visual input for color detection |

## System Design

```
┌─────────────────────────────────────────────────────────────────┐
│                  TurtleBot3 Platform (Physical/Simulated)       │
│  ┌──────────────┐  ┌──────────────┐  ┌─────────────────────┐   │
│  │ Power System │  │ Vision Sensor│  │  Drive Actuators    │   │
│  └──────┬───────┘  └──────┬───────┘  └──────────┬──────────┘   │
└─────────┼──────────────────┼───────────────────────┼────────────┘
          │                  │                       │
          ▼                  ▼                       ▼
    /battery_state    /camera/image_raw         /cmd_vel
          │                  │                       ▲
          │                  │                       │
┌─────────┴──────────────────┴───────────────────────┴────────────┐
│                   ROS 2 Messaging Infrastructure                 │
└─────┬──────────┬──────────────┬─────────────────┬───────────────┘
      │          │              │                 │
      ▼          ▼              ▼                 ▼
┌──────────┐ ┌────────────┐ ┌──────────────┐ ┌────────────────┐
│  Health  │ │   Vision   │ │Color Tracking│ │    Path        │
│  Monitor │ │  Processor │ │     Module   │ │   Planner      │
└─────┬─────┘ └─────┬──────┘ └──────┬───────┘ └────────────────┘
      │             │                │
      │             ▼                │
      │    turtlebot3/perception/*   │
      │                              │
      ▼                              │
turtlebot3/alerts                    │
                                     │
                                     ▼
                            Locomotion Control
```

## Module Organization

```
turtlebot3_automation/
├── Functional Modules
│   ├── maintenance/          # System vitals tracking
│   ├── perception/           # Neural network vision (YOLO)
│   ├── navigation/           # Autonomous path planning (Nav2)
│   └── custom_features/      # Color-based visual servoing
│
├── Infrastructure
│   ├── utils/                # Logging and path utilities
│   └── setup_automation/     # Installation tooling
│
└── Configuration Assets
    ├── config/               # Parameter definitions
    ├── launch/               # Startup configurations
    └── tests/                # Validation suites
```

## License Information

This software is distributed under the MIT License. Full terms are available in the [LICENSE](LICENSE) document.

## Contribution Guidelines

Community contributions are encouraged! Review [CONTRIBUTING.md](CONTRIBUTING.md) for development standards and submission procedures.

## Credits

- Foundation: ROS 2 Foxy distribution with Nav2 navigation framework
- Vision AI: YOLOv8 neural network by Ultralytics
- Target Platform: ROBOTIS TurtleBot3 mobile robot series

## Support Channels

Questions, bug reports, and feature requests can be submitted through GitHub Issues.
