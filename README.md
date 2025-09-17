# ARGUS: Autonomous Robotic Guardian Unit System

**ARGUS** is a ROS-based autonomous robot designed to ensure habitat safety and operational reliability for future lunar missions. In environments lacking GPS and under hazardous conditions, ARGUS acts as a vigilant guardian, reducing human workload and enhancing astronaut safety.

This project, developed for the Smart India Hackathon (SIH), is a prototype of an autonomous system capable of navigating a simulated lunar habitat, monitoring critical environmental parameters, and signaling alerts when anomalies are detected.

-----

## Features

  - **Autonomous Patrol:** ARGUS can autonomously navigate a predefined patrol route covering both indoor and outdoor sections of a lunar habitat.
  - **Mapping & Localization:** Utilizes the ROS navigation stack with `gmapping` for creating a 2D map of the environment and `amcl` for robust localization within that map.
  - **Environmental Monitoring:** Constantly monitors critical life-support parameters, such as oxygen levels and ambient temperature.
  - **Anomaly & Hazard Detection:** Identifies when environmental parameters go beyond safe operational thresholds.
  - **Alert System:** Provides clear, immediate alerts to the console when an anomaly is detected, specifying the nature of the hazard.
  - **Obstacle Avoidance:** Employs LiDAR data for real-time detection and avoidance of static and dynamic obstacles not present on the initial map.
  - **Simulation-First:** Fully developed and validated in a Gazebo 3D simulation environment, modeling a lunar habitat.

-----

## System Architecture

ARGUS is built on a modular ROS architecture, with each core functionality separated into its own package for clarity and scalability.

  - **`argus_description`**: Contains the URDF (Unified Robot Description Format) model of the ARGUS robot, defining its physical structure, joints, and sensor placements.
  - **`argus_gazebo`**: Includes the Gazebo world files for the simulated lunar habitat and the launch files required to spawn the robot in the simulation.
  - **`argus_navigation`**: Configures the powerful ROS Navigation Stack (`move_base`, `amcl`, `gmapping`) tailored for the ARGUS robot and the lunar environment.
  - **`argus_tasks`**: This is the core mission logic package. It contains the Python nodes responsible for:
      - `patrol_node.py`: Manages the sequence of patrol waypoints.
      - `monitoring_node.py`: Subscribes to sensor data and checks for anomalies.
      - `alert_node.py`: Listens for anomaly flags and triggers alerts.

-----

## Getting Started

### Prerequisites

  - **Ubuntu 20.04 LTS:** The recommended operating system.
  - **ROS Noetic Ninjemys:** The core robotics framework. [Installation Guide](http://wiki.ros.org/noetic/Installation/Ubuntu).
  - **Git:** For version control.
  - **Catkin Tools:** The ROS build system.

### Installation & Setup

1.  **Install ROS Noetic and its dependencies:**
    Follow the official ROS Noetic installation guide. Ensure you install the `ros-noetic-desktop-full` package.

2.  **Create a Catkin Workspace:**

    ```bash
    mkdir -p ~/argus_ws/src
    cd ~/argus_ws/
    catkin_make
    ```

3.  **Clone the Repository:**
    Clone this repository into the `src` directory of your workspace.

    ```bash
    cd ~/argus_ws/src
    git clone [Your Repository URL]
    ```

4.  **Install Dependencies:**
    Use `rosdep` to install all necessary package dependencies.

    ```bash
    cd ~/argus_ws
    rosdep install --from-paths src --ignore-src -r -y
    ```

5.  **Build the Workspace:**

    ```bash
    cd ~/argus_ws
    catkin_make
    ```

6.  **Source the Workspace:**
    You must source the setup file in every new terminal you open. For convenience, add it to your `.bashrc`.

    ```bash
    echo "source ~/argus_ws/devel/setup.bash" >> ~/.bashrc
    source ~/.bashrc
    ```

-----

## Usage

Follow these steps to run the full ARGUS mission simulation.

### 1\. Launch the Simulation Environment

This command starts the Gazebo simulator with the lunar habitat world and spawns the ARGUS robot inside it.

```bash
roslaunch argus_gazebo spawn_argus.launch
```

### 2\. Perform Mapping (One-Time Setup)

To enable autonomous navigation, you first need to create a map of the environment.

  - **Launch the mapping node:**
    ```bash
    roslaunch argus_navigation gmapping.launch
    ```
  - **Launch a teleop node to drive the robot:**
    ```bash
    rosrun teleop_twist_keyboard teleop_twist_keyboard.py
    ```
  - **Visualize in RViz:** Open RViz and add the `/map` topic display. Drive the robot slowly through the entire habitat (indoor and outdoor) until the map is complete.
  - **Save the map:**
    ```bash
    rosrun map_server map_saver -f ~/argus_ws/src/argus_gazebo/maps/lunar_habitat
    ```

### 3\. Run the Autonomous Patrol & Monitoring Mission

This is the main launch file. It starts the navigation system with your saved map and launches the custom task nodes for patrolling and monitoring.

```bash
roslaunch argus_tasks mission.launch
```

Once launched, ARGUS will begin its autonomous patrol. You can monitor its progress in RViz and see alert messages printed to the console if environmental anomalies are detected.

-----

## Project Structure

```bash
.
├── .gitignore
└── src/
    ├── description/
    │   ├── launch/
    │   ├── urdf/
    │   │   └── parts/
    │   ├── CMakeLists.txt
    │   └── package.xml
    │
    ├── gazebo/
    │   ├── launch/
    │   ├── worlds/
    │   ├── maps/CMakeLists.txt
    │   ├── CMakeLists.txt
    │   └── package.xml
    │
    ├── navigation/
    │   ├── launch/
    │   ├── params/
    │   ├── CMakeLists.txt
    │   └── package.xml
    │
    └── tasks/
        ├── launch/
        ├── scripts/
        ├── CMakeLists.txt
        └── package.xml
```

-----

## Technology Stack

  - **Operating System:** Ubuntu 20.04 LTS
  - **Robotics Framework:** ROS 1 Noetic Ninjemys
  - **Simulation:** Gazebo
  - **Visualization:** RViz
  - **Core Logic:** Python
  - **Version Control:** Git