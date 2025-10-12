# ARGUS: Autonomous Robotic Guardian Unit System

**ARGUS** is a ROS-based autonomous robot designed to ensure habitat safety and operational reliability for future lunar missions. In environments lacking GPS and under hazardous conditions, ARGUS acts as a vigilant guardian, reducing human workload and enhancing astronaut safety.

This project is a prototype of an autonomous system capable of navigating a simulated lunar habitat, monitoring critical environmental parameters, and signaling alerts when anomalies are detected.

---

## Features

-   **3D Mapping & Localization:** Utilizes a 3D LiDAR and the **RTAB-Map** SLAM package to generate detailed 3D point cloud maps of the environment and robustly track the robot's position within them.
-   **Realistic Simulation:** Operates in a custom Gazebo world that simulates the uneven terrain of the lunar surface using heightmaps and textures.
-   **Autonomous Patrol:** Capable of navigating a predefined patrol route across the challenging 3D lunar landscape.
-   **Environmental Monitoring & Alerting:** Monitors critical life-support parameters and signals alerts when anomalies are detected.
-   **Advanced Obstacle Avoidance:** Employs 3D point cloud data for real-time detection and avoidance of complex obstacles like rocks and crater edges.
-   **Simulation-First:** Fully developed and validated in a Gazebo 3D simulation environment, modeling a lunar habitat.

---

## System Architecture

ARGUS is built on a modular ROS architecture, with each core functionality separated into its own package for clarity and scalability. The ROS packages are located within the `src/` directory of this repository.

-   **`description`**: Contains the URDF (Unified Robot Description Format) model of the ARGUS robot.
-   **`gazebo`**: Includes the Gazebo world files for the simulated lunar habitat and the launch files required to spawn the robot.
-   **`navigation`**: Configures the ROS Navigation Stack (`move_base`, `amcl`, `gmapping`).
-   **`tasks`**: Contains the core mission logic nodes for patrolling, monitoring, and alerting.

---

## Getting Started

### Prerequisites

-   **Ubuntu 20.04 LTS**
-   **ROS Noetic Ninjemys:** [Installation Guide](http://wiki.ros.org/noetic/Installation/Ubuntu).
-   **Git:** For version control.
-   **Catkin Tools:** The recommended ROS build system.
    ```bash
    sudo apt-get install python3-catkin-tools
    ```
-   **RTAB-Map for ROS:** You must install the 3D SLAM package.
    ```bash
    sudo apt-get install ros-noetic-rtabmap-ros
    ```


### Installation & Setup

These instructions are tailored to the specific structure of this repository.

1.  **Create a Catkin Workspace:**
    ```bash
    mkdir -p ~/argus_ws/src
    cd ~/argus_ws/
    ```

2.  **Clone the Repository:**
    ```bash
    cd ~/argus_ws/src
    git clone [https://github.com/Aaryan-khairnar/ARGUS/](https://github.com/Aaryan-khairnar/ARGUS/)
    ```

3.  **Configure the Source Directory (One-Time Step):**
    This command tells the build system where your packages are located and only needs to be run once.
    ```bash
    cd ~/argus_ws
    catkin config --source src/ARGUS/src
    ```

4.  **Install Dependencies:**
    This command will install any other ROS packages your project needs.
    ```bash
    cd ~/argus_ws
    rosdep install --from-paths src/ARGUS/src --ignore-src -r -y
    ```

5.  **Build the Workspace:**
    Use the `catkin build` command to compile your project.
    ```bash
    cd ~/argus_ws
    catkin build
    ```

6.  **Source the Workspace:**
    Add the workspace to your environment.
    ```bash
    echo "source ~/argus_ws/devel/setup.bash" >> ~/.bashrc
    source ~/.bashrc
    ```

---

## Usage

This workflow is divided into two main phases: creating the realistic world and then mapping it in 3D.

### Phase 1: Create the Lunar World & Upgrade the Robot

First, we need to create the environment and give our robot the right sensor to perceive it.

1.  **Create the Moon Surface:** Follow the tutorial to create the `moon_surface.world` file using the provided heightmap and texture images. Make sure to update your `spawn.launch` file to load this new world.

2.  **Upgrade the Robot's URDF:** Open your `argus.urdf.xacro` file and add the 3D LiDAR sensor code block. This gives your robot the hardware it needs to see in 3D.

### Phase 2: Perform 3D Mapping with RTAB-Map

Now that you have a 3D world and a 3D-capable robot, you can begin mapping.

1.  **Launch the Simulation Environment (Terminal 1):**
    This starts Gazebo with your custom moon world and spawns the ARGUS robot, now equipped with its 3D LiDAR.
    ```bash
    roslaunch argus_gazebo spawn.launch
    ```

2.  **Launch the 3D SLAM Node (Terminal 2):**
    This command starts the RTAB-Map software in mapping mode. It will automatically start listening for the 3D LiDAR data.
    ```bash
    roslaunch argus_navigation rtabmap_mapping.launch
    ```

3.  **Launch the Keyboard Controller (Terminal 3):**
    This allows you to drive the robot around manually.
    ```bash
    rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/argus/cmd_vel
    ```

4.  **Visualize and Build the Map:**
    The `rtabmap_mapping.launch` file should have automatically opened the RTAB-Map visualization tool. In this window, you will see the 3D point cloud map being built in real-time as you drive.

    Slowly and carefully drive your robot around the entire lunar surface. Your goal is to cover all the terrain to create a complete and detailed 3D map.

5.  **Save the Map:**
    When you are finished mapping, RTAB-Map automatically saves the map to a database file located at `~/.ros/rtabmap.db`. You can simply close all the terminals by pressing `Ctrl+C`.

### Phase 3: Autonomous Navigation (Future Goal)

Once the map is saved, the next step in the project will be to use it for autonomous navigation. This involves running RTAB-Map in "localization mode" and using `move_base` to send navigation goals.

```bash
# Example command for the future
roslaunch argus_tasks mission.launch
```

-----

## Project Structure

```
ARGUS/
├── README.md
└── src/
    ├── description/
    │   ├── launch/
    │   ├── urdf/
    │   ├── CMakeLists.txt
    │   └── package.xml
    │
    ├── gazebo/
    │   ├── launch/
    │   ├── worlds/
    │   ├── maps/
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
  - **Build System:** Catkin Tools
  - **Simulation:** Gazebo
  - **Visualization:** RViz & RTAB-Map GUI
  - **Core Logic:** Python
  - **Version Control:** Git
