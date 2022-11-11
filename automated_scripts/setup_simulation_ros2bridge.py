# ----------------------------------------------------------------------------
# Created By   : Ali Kamberi & Guilherme Vicentini Briner
# Company      : Zurich UAS Racing
# Created Date : 23.04.2022
# version      : '1.0'
# ---------------------------------------------------------------------------
""" this script helps to simplify the interaction with the game engine 
and the ros bridge and allows for easier execution of the simulation. """
# ---------------------------------------------------------------------------
# Imports
# ---------------------------------------------------------------------------
from email.header import decode_header
from pathlib import Path
import os
import signal
import subprocess
import pwd
import re
from tkinter import E
from turtle import bgcolor


class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


_home_dir = "/home/sim"
_active_user = "sim"
_sim_pid = None
_ros2_pid = None
_ros2_bags_pid = None
_cwd = None
# REGEX_FOR_BYTE = "^b('|\")|('|\")$"
REGEX_FOR_PID = r"\n\d+"
ROS_GALACTIC_SOURCE_CMD = "source /opt/ros/galactic/setup.bash; "
_ros_bags_topics = "/control_command /fsds/cam1 /fsds/cam2 /gps /gss /imu /lidar/Lidar1 /lidar/Lidar2 /testing_only/extra_info /testing_only/odom /testing_only/track"
_ros_bags_folder = "saved_simulation_runs"
_ros2_bridge_optional_params = "manual_mode:=false rviz:=false plot:=false track_name:='A'"
_ros2_bridge_plots_active = False
_game_engine_folder_ros_bridge = "/Formula-Student-Driverless-Simulator/ros2"
_game_engine_folder = "/Formula-Student-Driverless-Simulator"
_game_engine_binary_folder = "/zur-v1.0.0"
_game_engine_binary_script = "ZUR.sh"
_simulation_ros2_script_helpers_folder = "/Formula-Student-Driverless-Simulator/automated_scripts/helpers"
ROS2_BRIDGE_BUILD_SH = "ros2_bridge_build.sh"
ROS2_BRIDGE_START_SH = "ros2_bridge_start.sh"
ROS2_BRIDGE_RESET_CAR_SH = "ros2_bridge_reset_car.sh"
ROS2_BRIDGE_ROS_BAG_RECORD_SH = "ros2_bridge_bags_start_record.sh"
ROS2_BRIDGE_LAUNCH_CMD = "ros2 launch fsds_ros2_bridge fsds_ros2_bridge.launch.py"

ACTIONS = f"""{bcolors.OKGREEN}
 (0): exit script, terminate ros2bridge & game engine
 (1): start game engine
 (2): stop game engine
 (3): build ros2 bridge
 (4): start ros 2 bridge
 (5): stop ros2 bridge
 (6): clean ros2 bridge
 (7): record rosbags topics
 (8): list all recorded simulation runs
 (9): reset car and plots
 (98): print script configurations
 (99): change user which contains game engine and ros2_bridge in homedir (active user: {_active_user})
 {bcolors.ENDC}
"""


def main():
    global _cwd, _home_dir
    print(f"{bcolors.OKCYAN}Please enter the number of the following actions you want to do:{bcolors.ENDC}")
    _cwd = os.getcwd()
    _home_dir = str(Path.home())
    keep_script_running = True

    while keep_script_running:
        print(ACTIONS)
        selected_action = input(f"{bcolors.BOLD}Please enter action number: {bcolors.ENDC}")
        if selected_action == "1":
            # start game engine
            game_engine_start()
        elif selected_action == "2":
            # stop game engine
            game_engine_terminate()
        elif selected_action == "3":
            # build ros2 bridge
            ros2_bridge_build()
        elif selected_action == "4":
            # build and start ros2 bridge
            optional_params = ros2_bridge_prepare_optional_params()
            ros2_bridge_start(optional_params=optional_params)
        elif selected_action == "5":
            # stop ros2 bridge
            ros2_bridge_terminate()
        elif selected_action == "6":
            # clean ros2 bridge
            ros2_bridge_clean()
        elif selected_action == "7":
            # start ros bags and record user defined topics (default: all topics)
            ros2_bags_check_and_create_folder()
            ros2_bags_start_record()
        elif selected_action == "8":
            # list recorded ros bags
            ros2_bag_list()
        elif selected_action == "9":
            # reset car and plots in simulation
            game_engine_reset_car_and_plots()
        elif selected_action == "0":
            print(f"{bcolors.OKCYAN}exiting script and terminating ros2bridge & game engine{bcolors.ENDC}")
            keep_script_running = False
            ros2_bridge_terminate()
            game_engine_terminate()
            exit()
        elif selected_action == "98":
            print_configuration()
        elif selected_action == "99":
            print(f"{bcolors.OKCYAN}Please enter the user which contains the directories with the game engine and the ros2_bridge: {bcolors.ENDC}")
            username = str(input(f"{bcolors.BOLD}Username: {bcolors.ENDC}"))
            change_user(username)

def game_engine_start():
    global _sim_pid
    print(f"{bcolors.OKCYAN}starting game engine{bcolors.ENDC}")
    binary_dir = _home_dir + _game_engine_binary_folder
    os.chdir(binary_dir)
    if Path(_game_engine_binary_script).is_file():
        cmd = f"/bin/bash ./{_game_engine_binary_script} &"
        sim_process = subprocess.Popen(
            cmd,
            stdout=subprocess.DEVNULL,
            shell=True,
            preexec_fn=os.setsid)
        _sim_pid = os.getpgid(sim_process.pid)
    else:
        print(f"{bcolors.FAIL}could not locate executable script for game engine. Please check the path to the script\nused path: {binary_dir}\nfilename: {_game_engine_binary_script}{bcolors.ENDC}")

def game_engine_terminate():
    global _sim_pid
    if _sim_pid:
        try:
            os.killpg(_sim_pid, signal.SIGTERM)
            print(f"{bcolors.OKCYAN}terminated simulation{bcolors.ENDC}")
            _sim_pid = None
        except ProcessLookupError:
            print(f"{bcolors.FAIL}simulation could not be terminated because the process was not found. please terminate it manually{bcolors.ENDC}")
    else:
        print(f"{bcolors.FAIL}no process id is defined for the game engine. Please terminate it manually.{bcolors.ENDC}")
    return

def game_engine_reset_car_and_plots():
    script_folder = _home_dir + _simulation_ros2_script_helpers_folder
    sim_folder = _home_dir + _game_engine_folder_ros_bridge
    cmd = f"/bin/bash {script_folder}/{ROS2_BRIDGE_RESET_CAR_SH} '{sim_folder}'"
    cmd += " 'plots=True'" if _ros2_bridge_plots_active else ''
    ret = subprocess.run(
        cmd,
        capture_output=True,
        shell=True
    )
    print(f"{bcolors.OKCYAN}resetting car in simulation{bcolors.ENDC}")
    print(ret.stdout.decode())

def ros2_bridge_build():
    print(f"{bcolors.OKCYAN}building ros2_bridge{bcolors.ENDC}")
    ros_bridge_folder = _home_dir + _game_engine_folder_ros_bridge
    script_folder = _home_dir + _simulation_ros2_script_helpers_folder

    cmd = f"/bin/bash {script_folder}/{ROS2_BRIDGE_BUILD_SH} '{ros_bridge_folder}'"
    ret = subprocess.run(
        cmd,
        capture_output=True,
        shell=True)
    print(ret.stdout.decode())
    return

def ros2_bridge_start(optional_params=None):
    global _ros2_pid
    print(f"{bcolors.OKCYAN}starting ros2_bridge{bcolors.ENDC}")
    sim_folder = _home_dir + _game_engine_folder_ros_bridge
    script_folder = _home_dir + _simulation_ros2_script_helpers_folder
    os.chdir(script_folder)

    cmd = f"gnome-terminal --title='ros2_bridge' --tab -- bash -c '/bin/bash {script_folder}/{ROS2_BRIDGE_START_SH} {sim_folder} \"{optional_params if optional_params else ''}\"'"
    ros2_bridge_process = subprocess.Popen(
        cmd,
        shell=True)
    ros2_bridge_parent_pid = get_pid_by_executed_script(ROS2_BRIDGE_START_SH)
    _ros2_pid = get_child_pid_by_parent_pid(ros2_bridge_parent_pid)
    print("PROCESS ID: " + str(_ros2_pid))

def get_pid_by_executed_script(shell_script):
    # the sleep is necessary to get the pid from the script itself and not the gnome-terminal process
    cmd = f"sleep 5; pgrep --list-full -u {_active_user} | grep '{shell_script}'"
    ret = subprocess.run(
        cmd,
        capture_output=True,
        shell=True
    )
    decoded_ret = ret.stdout.decode()
    pid_list = re.findall(REGEX_FOR_PID, decoded_ret)
    if pid_list and len(pid_list) >= 1:
        pid = int(pid_list[0])
        print(f"{bcolors.OKCYAN}parent id: {pid}{bcolors.ENDC}")
        return pid
    print(f"{bcolors.FAIL}could not get process id of ros2 bridge terminal.\nThe terminal needs to be terminated manually later!{bcolors.ENDC}")
    return None

def get_child_pid_by_parent_pid(ppid):
    cmd = f"pgrep -P {ppid}"
    ret = subprocess.run(
        cmd,
        capture_output=True,
        shell=True
    )
    child_pid = ret.stdout.decode()
    print(f"{bcolors.OKCYAN}child id: {child_pid}{bcolors.ENDC}")
    if child_pid and int(child_pid):
        return int(child_pid)
    return None

def ros2_bridge_terminate():
    print(f"{bcolors.OKCYAN}terminating ros2_bridge{bcolors.ENDC}")
    global _ros2_pid, _ros2_bridge_plots_active
    if _ros2_pid:
        try:
            cmd = f"pkill -SIGINT -P {_ros2_pid}"
            ret = subprocess.run(
                cmd,
                shell=True
            )
            print(f"{bcolors.OKGREEN}terminated ros2 bridge{bcolors.ENDC}")
            _ros2_pid = None
            _ros2_bridge_plots_active = False
        except ProcessLookupError:
            print(f"{bcolors.WARNING}ros2 bridge could not be terminated because the process was not found. please terminate it manually{bcolors.ENDC}")
    else:
        print(f"{bcolors.FAIL}no process id is defined for the game engine. Please terminate it manually.{bcolors.ENDC}")
    return

def ros2_bridge_clean():
    print(f"{bcolors.OKCYAN}cleaning ros2 bridge{bcolors.ENDC}")
    sim_folder = _home_dir + _game_engine_folder_ros_bridge
    os.chdir(sim_folder)
    if Path("src").is_dir():
        cmd = "rm -rf build/ install/ log/"
        subprocess.run(
            cmd, 
            shell=True)
    return

def ros2_bridge_prepare_optional_params():
    global _ros2_bridge_optional_params, _ros2_bridge_plots_active
    prettier_optional_params = _ros2_bridge_optional_params.replace(" ", "\n")
    print(f"{bcolors.BOLD}This is the current configuration for the optional params. Do you want to change it?{bcolors.ENDC}\n{bcolors.OKGREEN}{prettier_optional_params}{bcolors.ENDC}")
    do_changes = input("y/n?: ")
    if str(do_changes) and str(do_changes).lower() == "y":
        new_optional_params = ""
        # manual mode
        print(f"{bcolors.BOLD}Do you want to enable manual_mode?{bcolors.ENDC}")
        new_optional_params += "manual_mode:=true " if str(input("y/n?: ")).lower() == "y" else ""
        # rviz
        print(f"{bcolors.BOLD}Do you want to enable rviz?{bcolors.ENDC}")
        new_optional_params += "rviz:=true " if str(input("y/n?: ")).lower() == "y" else ""
        # plots
        print(f"{bcolors.BOLD}Do you want to enable plots?{bcolors.ENDC}")
        if str(input("y/n?: ")).lower() == "y":
            new_optional_params += "plot:=true "
            _ros2_bridge_plots_active = True
        return new_optional_params
    return None

def ros2_bags_start_record():
    global _ros2_bags_pid
    ros_bags_folder = _home_dir + _game_engine_folder + _ros_bags_folder
    sim_folder = _home_dir + _game_engine_folder_ros_bridge
    script_folder = _home_dir + _simulation_ros2_script_helpers_folder

    os.chdir(script_folder)
    # cmd = f"{ROS_GALACTIC_SOURCE_CMD} . {sim_folder}/install/local_setup.bash; cd {sim_folder}/{_ros_bags_folder}; ros2 bag record -a" # {_ros_bags_topics}"
    # cmd = f"gnome-terminal --tab -- bash -c '{ROS_GALACTIC_SOURCE_CMD} ros2 bag record -a'"
    cmd = f"gnome-terminal --tab -- bash -c '/bin/bash {script_folder}/{ROS2_BRIDGE_ROS_BAG_RECORD_SH} '{ros_bags_folder}"
    ros2_bags_process = subprocess.Popen(
        cmd,
        shell=True,
        )
    ros2_bags_parent_pid = get_pid_by_executed_script(ROS2_BRIDGE_ROS_BAG_RECORD_SH)
    _ros2_bags_pid = get_child_pid_by_parent_pid(ros2_bags_parent_pid)
    print("PROCESS ID: " + str(_ros2_bags_pid))

def ros2_bags_check_and_create_folder():
    ros_bags_folder = f"{_home_dir}{_game_engine_folder}/{_ros_bags_folder}"
    if not Path(ros_bags_folder).is_dir():
        try:
            os.mkdir(ros_bags_folder)
        except OSError:
            print(f"{bcolors.FAIL}Could not create folder for ros bags{bcolors.ENDC}")

def ros2_bag_list():
    ros_bags_folder = f"{_home_dir}{_game_engine_folder}/{_ros_bags_folder}"
    cmd = f"ls -l {ros_bags_folder}; nautilus {ros_bags_folder} &"
    ret = subprocess.run(
        cmd,
        capture_output=True,
        shell=True
    )
    print(f"path containing simulation runs / recorded ros bags: \n{ros_bags_folder}")
    print(ret.stdout.decode())

def print_configuration():
    ros_bags_folder = f"{_home_dir}{_game_engine_folder}/{_ros_bags_folder}"
    current_dir = os.getcwd()
    configuration = f"""
    current dir                        : {current_dir}
    active user                        : {_active_user}
    current game engine pid            : {_sim_pid}
    current ros2 bridge pid            : {_ros2_pid}
    current ros2 bridgeoptional params : {_ros2_bridge_optional_params}
    ros bags folder                    : {ros_bags_folder}
    ros bags topics                    : {_ros_bags_topics}
    """
    print(configuration)

def change_user(username):
    global _active_user, _home_dir
    try:
        pwd.getpwnam(username)
        _active_user = username
        _home_dir = f"/home/{username}"
        print(f"Username and home dir changed to:\nUsername: {_active_user}\nHome dir: {_home_dir}\n")
    except KeyError:
        print(f"user {username} does not exist")

def change_to_home_dir():
    os.chdir(_home_dir)

def change_to_work_dir():
    os.chdir(_cwd)

if __name__ == "__main__":
    main()

