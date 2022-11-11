from turtle import clear
from sensor_msgs.msg import Imu, NavSatFix
from nav_msgs.msg import Odometry
from fs_msgs.msg import Track, ControlCommand
from fszhaw_msgs.msg import CurrentPosition, PlannedTrajectory
from typing import List, Tuple


class ImuStatistics():
    def __init__(self) -> None:
        self.time = []
        self.lin_accel_x = []
        self.lin_accel_y = []
        self.max_element_num = 100

    def handle_msg(self, msg: Imu) -> None:
        seconds = msg.header.stamp.sec
        x = msg.linear_acceleration.x
        y = msg.linear_acceleration.y

        self.time.append(seconds)
        self.lin_accel_x.append(x)
        self.lin_accel_y.append(y)
    
    def get_time(self) -> List[int]:
        return self.time

    def get_lin_accel_x(self) -> List[float]:
        return self.lin_accel_x
    
    def get_lin_accel_y(self) -> List[float]:
        return self.get_lin_accel_y

    def get_last_elements(self) -> Tuple:
        return self.time[-self.max_element_num:], self.lin_accel_x[-self.max_element_num:], self.lin_accel_y[-self.max_element_num:]

    def reset(self):
        self.time.clear()
        self.lin_accel_x.clear()
        self.lin_accel_y.clear()


class OdomStatistics():
    def __init__(self) -> None:
        self.position_x = []
        self.position_y = []
        self.max_element_num = 100

    def handle_msg(self, msg: Odometry) -> None:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        self.position_x.append(x)
        self.position_y.append(y)
    
    def get_position_x(self) -> List[float]:
        return self.position_x
    
    def get_position_y(self) -> List[float]:
        return self.position_y

    def get_last_elements(self) -> Tuple:
        return self.position_x[-self.max_element_num:], self.position_y[-self.max_element_num:]

    def reset(self):
        self.position_x.clear()
        self.position_y.clear()


class GpsStatistics():
    def __init__(self) -> None:
        self.position_x = []
        self.position_y = []
        self.max_element_num = 100

    def handle_msg(self, msg: NavSatFix) -> None:
        x = msg.longitude
        y = msg.latitude

        self.position_x.append(x)
        self.position_y.append(y)
    
    def get_position_x(self) -> List[float]:
        return self.position_x
    
    def get_position_y(self) -> List[float]:
        return self.position_y

    def get_last_elements(self) -> Tuple:
        return self.position_x[-self.max_element_num:], self.position_y[-self.max_element_num:]

    def reset(self):
        self.position_x.clear()
        self.position_y.clear()

class TrackStatistics():
    def __init__(self) -> None:
        self.blue_cones_x = []
        self.blue_cones_y = []

        self.yellow_cones_x = []
        self.yellow_cones_y = []
        
        self.big_orange_cones_x = []
        self.big_orange_cones_y = []

        self.small_orange_cones_x = []
        self.small_orange_cones_y = []

        self.max_element_num = 100

    def handle_msg(self, msg: Track) -> None:
        for cone in msg.track:
            if cone.color == 0:
                self.blue_cones_x.append(cone.location.x)
                self.blue_cones_y.append(cone.location.y)
            elif cone.color == 1:
                self.yellow_cones_x.append(cone.location.x)
                self.yellow_cones_y.append(cone.location.y)
            elif cone.color == 2:
                self.big_orange_cones_x.append(cone.location.x)
                self.big_orange_cones_y.append(cone.location.y)
            elif cone.color == 3:
                self.small_orange_cones_x.append(cone.location.x)
                self.small_orange_cones_y.append(cone.location.y)
            else:
                pass        
    
    def get_blue_cones(self) -> Tuple:
        return self.blue_cones_x, self.blue_cones_y

    def get_yellow_cones(self) -> Tuple:
        return self.yellow_cones_x, self.yellow_cones_y

    def get_big_orange_cones(self) -> Tuple:
        return self.big_orange_cones_x, self.big_orange_cones_y

    def get_small_orange_cones(self) -> Tuple:
        return self.small_orange_cones_x, self.small_orange_cones_y


class ControlStatistics():
    def __init__(self) -> None:
        self.seconds = []
        self.steering = []
        self.throttle = []
        self.brake = []
        self.max_element_num = 100

    def handle_msg(self, msg: ControlCommand) -> None:
        sec = msg.header.stamp.sec
        throttle = msg.throttle
        steering = msg.steering
        brake = msg.brake

        self.seconds.append(sec)
        self.throttle.append(throttle)
        self.steering.append(steering)
        self.brake.append(brake)

    def get_sec(self) -> List[float]:
        return self.seconds
    
    def get_throttle(self) -> List[float]:
        return self.throttle
    
    def get_steering(self) -> List[float]:
        return self.steering

    def get_steering(self) -> List[float]:
        return self.brake

    def get_last_elements(self) -> Tuple:
        return self.seconds[-self.max_element_num:], self.throttle[-self.max_element_num:], self.steering[-self.max_element_num:], self.brake[-self.max_element_num:]
    
    def reset(self):
        self.seconds.clear()
        self.steering.clear()
        self.throttle.clear()
        self.brake.clear()

class PositionStatistics():
    def __init__(self) -> None:
        self.position_x = []
        self.position_y = []
        self.max_element_num = 100

    def handle_msg(self, msg: CurrentPosition) -> None:
        x = msg.vehicle_position_x
        y = msg.vehicle_position_y

        self.position_x.append(x)
        self.position_y.append(y)
    
    def get_position_x(self) -> List[float]:
        return self.position_x
    
    def get_position_y(self) -> List[float]:
        return self.position_y

    def get_last_elements(self) -> Tuple:
        return self.position_x[-self.max_element_num:], self.position_y[-self.max_element_num:]
    
    def reset(self):
        self.position_x.clear()
        self.position_y.clear()


class TrajectoryStatistics():
    def __init__(self) -> None:
        self.position_x = []
        self.position_y = []
        self.max_element_num = 100

    def handle_msg(self, msg: PlannedTrajectory) -> None:
        x = msg.target_x
        y = msg.target_y

        self.position_x.append(x)
        self.position_y.append(y)
    
    def get_position_x(self) -> List[float]:
        return self.position_x
    
    def get_position_y(self) -> List[float]:
        return self.position_y

    def get_last_elements(self) -> Tuple:
        return self.position_x[-self.max_element_num:], self.position_y[-self.max_element_num:]

    def reset(self):
        self.position_x.clear()
        self.position_y.clear()


