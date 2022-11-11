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

1. Clone this repository

2. Download the Unreal Engine 4 Project from the public iCloud link (see *UE4Project.md* for more details)

   https://www.icloud.com/iclouddrive/0dc8HNweV5I4B9NP1bbjUUj1w#UE4Project

3. Change to the *automated_scripts* directory

   `cd automated_scripts`

4. Execute the install script, which will set up ROS2 Foxy & Galactic, the customized Formula Student Driverless Simulator for ZUR and additional NVidia Tools

   `./install_simulation_ros2foxy.sh`

5. Run the simulator via the Python Script

   `python3 setup_simulation_ros2bridge.py`

# Available Vehicles

`"DefaultCar": { "PawnBP": "Class'/AirSim/VehicleAdv/Cars/TechnionCar/TechnionCarPawn.TechnionCarPawn_C'"`

`"DefaultCar": { "PawnBP": "Class'/AirSim/VehicleAdv/Cars/ZurCar/ZurCarPawn.ZurCarPawn_C'"`
