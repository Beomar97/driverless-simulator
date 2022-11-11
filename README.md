# Driverless Simulator

> The Simulator has been customized for the needs of ZUR by Ali Kamberi and Guilherme Vicentini Briner. The original fork can be found on the GitHub Enterprise Server of the Zurich University of Applied Sciences (ZHAW): https://github.zhaw.ch/FSZHAW/Simulation_Tool (Access restricted to students and faculty)

This project is a hard fork from the amazing [FS-Driverless Simulator](https://github.com/FS-Driverless/Formula-Student-Driverless-Simulator) project.

We chose to do a hard fork because we wanted to
- Do a lot of changes in a very short time
- Not be slowed down by git submodule problems

Since we are not planning to keep this fork up to date with upstream or PR changes here, we see no big disadvantages.

All irrelevant code in this project has been removed, so new developers only see code that is actually in use.

Relevant documentation has been moved to /docs, all other tutorials and docs have been deleted.

# Getting Started

For both setups, at least one machine with a recent Nvidia GPU with Vulkan support is necessary!

[Recommended System Requirements](https://fs-driverless.github.io/Formula-Student-Driverless-Simulator/v2.1.0/getting-started/) for the computer running the simulator:

- 8 core 2.3Ghz CPU
- 12 GB memory
- 30GB free SSD storage (120GB when building the unreal project from source)
- Recent NVidia card with Vulkan support and 3 GB of memory. (You can check the video card drivers by running `vulkaninfo`). Different brand video cards might work but have not been tested.

**Setup #1: One Linux Machine running both the Simulator and the Autonomous System**

1. Clone this repository in your [**home directory**](https://fs-driverless.github.io/Formula-Student-Driverless-Simulator/v2.1.0/getting-started-with-ros/)

   > **THE REPO HAS TO BE CLONED IN THE HOME DIRECTORY!**. So the repo location should be `$HOME/Formula-Student-Driverless-Simulator`. Why you ask? Because we couldn't get relative paths in the C++ code to work so now we have hard-coded some paths to the home directory. I know yes it is ugly but it works. If you are bothered by it I would welcome you to open a pr with a fix.

2. Download the Unreal Engine 4 Project from the public iCloud link (see *UE4Project.md* for more details)

   https://www.icloud.com/iclouddrive/0dc8HNweV5I4B9NP1bbjUUj1w#UE4Project

3. Change to the *automated_scripts* directory

   `cd automated_scripts`

4. Execute the install script, which will set up ROS2 Foxy & Galactic, the customized Formula Student Driverless Simulator for ZUR and additional NVidia Tools

   `./install_simulation_ros2foxy.sh`

5. Run the simulator via the Python Script

   `python3 setup_simulation_ros2bridge.py`

**Setup #2: Two Machines, one running the Autonomous System & ROS Bridge and one running the Simulator**

1. tbd

# Available Vehicles

`"DefaultCar": { "PawnBP": "Class'/AirSim/VehicleAdv/Cars/TechnionCar/TechnionCarPawn.TechnionCarPawn_C'"`

`"DefaultCar": { "PawnBP": "Class'/AirSim/VehicleAdv/Cars/ZurCar/ZurCarPawn.ZurCarPawn_C'"`
