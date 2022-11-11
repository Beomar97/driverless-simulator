from cProfile import label
from turtle import color
import rclpy
from rclpy.node import Node
from typing import List, Tuple
from sensor_msgs.msg import Imu, NavSatFix
from fs_msgs.msg import Track, ControlCommand
from fs_msgs.srv import Reset
from fszhaw_msgs.msg import CurrentPosition, PlannedTrajectory
from nav_msgs.msg import Odometry
from .statistics import ImuStatistics, OdomStatistics, GpsStatistics, TrackStatistics, ControlStatistics, PositionStatistics, TrajectoryStatistics
import matplotlib.pyplot as plt 
import matplotlib
from matplotlib.gridspec import GridSpec

class StatSubscriber(Node):
    def __init__(self):
        super().__init__('statistics_subscriber')
        self.imu_subscription = self.create_subscription(
            Imu,
            'imu',
            self.imu_listener_callback,
            10)
        self.imu_subscription  # prevent unused variable warning
        self.imu_handler = ImuStatistics()

        self.odom_subscription = self.create_subscription(
            Odometry,
            'testing_only/odom',
            self.odom_listener_callback,
            10)
        self.odom_subscription  # prevent unused variable warning
        self.odom_handler = OdomStatistics()

        self.gps_subscription = self.create_subscription(
            NavSatFix,
            'gps',
            self.gps_listener_callback,
            10)
        self.gps_subscription  # prevent unused variable warning
        self.gps_handler = GpsStatistics()

        self.track_subscription = self.create_subscription(
            Track,
            'testing_only/track',
            self.track_listener_callback,
            10)
        self.track_subscription  # prevent unused variable warning
        self.track_handler = TrackStatistics()

        self.control_subscription = self.create_subscription(
            ControlCommand,
            'control_command',
            self.control_listener_callback,
            10)
        self.control_subscription  # prevent unused variable warning
        self.control_handler = ControlStatistics()

        self.position_subscription = self.create_subscription(
            CurrentPosition,
            'current_position',
            self.position_listener_callback,
            10)
        self.position_subscription  # prevent unused variable warning
        self.position_handler = PositionStatistics()

        self.trajectory_subscription = self.create_subscription(
            PlannedTrajectory,
            'planned_trajectory',
            self.trajectory_listener_callback,
            10)
        self.trajectory_subscription  # prevent unused variable warning
        self.trajectory_handler = TrajectoryStatistics()

        self.service = self.create_service(Reset, 'plot_reset', self.reset_callback)

        fig = plt.figure(figsize=(10,5))

        gs = GridSpec(nrows=4, ncols=3)

        self.controls_plot = fig.add_subplot(gs[0,0])
        self.steering_plot = fig.add_subplot(gs[1,0])
        self.accel_plot = fig.add_subplot(gs[2,0])
        self.gps_plot = fig.add_subplot(gs[3,0])

        self.position_plot = fig.add_subplot(gs[:, 1])
        self.track_plot = fig.add_subplot(gs[:, 2])


    def mypause(self, interval):
        backend = plt.rcParams['backend']
        if backend in matplotlib.rcsetup.interactive_bk:
            figManager = matplotlib._pylab_helpers.Gcf.get_active()
            if figManager is not None:
                canvas = figManager.canvas
                if canvas.figure.stale:
                    canvas.draw()
                canvas.start_event_loop(interval)
                return

    def imu_listener_callback(self, msg):
        self.imu_handler.handle_msg(msg)
        time, accel_x, accel_y = self.imu_handler.get_last_elements()

        self.accel_plot.cla()
        self.accel_plot.plot(time, accel_x, label='Longitudinal')
        self.accel_plot.plot(time, accel_y, label='Lateral')
        self.accel_plot.legend(loc=2)
        self.accel_plot.set_title("IMU Car's acceleration")
        self.accel_plot.set_xlabel('Seconds')
        self.accel_plot.set_ylabel('Acceleration (m/s^2)')

        plt.draw()
        self.mypause(0.0001)

    def track_listener_callback(self, msg):
        self.track_handler.handle_msg(msg)
        blue_x, blue_y = self.track_handler.get_blue_cones()
        yellow_x, yellow_y = self.track_handler.get_yellow_cones()
        big_orange_x, big_orange_y = self.track_handler.get_big_orange_cones()
        small_orange_x, small_orange_y = self.track_handler.get_small_orange_cones()

        car_pos_x = self.odom_handler.get_position_x()
        car_pos_y = self.odom_handler.get_position_y()

        planned_pos_x = self.trajectory_handler.get_position_x()
        planned_pos_y = self.trajectory_handler.get_position_y()

        self.track_plot.cla()
        self.track_plot.plot(blue_x, blue_y, 'b^')
        self.track_plot.plot(yellow_x, yellow_y, 'y^')
        self.track_plot.plot(big_orange_x, big_orange_y, 'r^' )
        self.track_plot.plot(small_orange_x, small_orange_y, 'c^')
        self.track_plot.plot(car_pos_x, car_pos_y, 'k.', label="Car's position")
        self.track_plot.plot(planned_pos_x, planned_pos_y, 'g.', label="Planned Trajectory")
        self.track_plot.legend(loc=2)


        self.track_plot.set_title('Track')

        plt.draw()
        self.mypause(0.0001)

    def control_listener_callback(self, msg):
        self.control_handler.handle_msg(msg)
        time, throttle, steering, brake = self.control_handler.get_last_elements()

        self.controls_plot.cla()
        self.controls_plot.plot(time, throttle, label="throttle")
        self.controls_plot.plot(time, brake, label="brake")

        self.controls_plot.set_title("Car's commands")

        self.steering_plot.cla()
        self.steering_plot.plot(time, steering)

        self.steering_plot.set_title("Car's Steering")
        self.steering_plot.set_ylabel('Steering')
        self.steering_plot.set_ylim([-1,1])

        plt.draw()
        self.mypause(0.0001)


    def gps_listener_callback(self, msg):
        self.gps_handler.handle_msg(msg)
        gps_pos_x = self.gps_handler.get_position_x()
        gps_pos_y = self.gps_handler.get_position_y()


        self.gps_plot.cla()
        self.gps_plot.plot(gps_pos_x, gps_pos_y, color='k', label='GPS Position')
        self.gps_plot.set_title("Car's gps position")
        self.gps_plot.set_xlabel('latitude')
        self.gps_plot.set_ylabel('longitude')

        plt.draw()
        self.mypause(0.0001)

    def odom_listener_callback(self, msg):
        self.odom_handler.handle_msg(msg)

        pos_x = self.trajectory_handler.get_position_x()
        pos_y = self.trajectory_handler.get_position_y()

        odom_pos_x = self.odom_handler.get_position_x()
        odom_pos_y = self.odom_handler.get_position_y()

        calc_pos_x = self.position_handler.get_position_x()
        calc_pos_y = self.position_handler.get_position_y()

        
        self.position_plot.cla()
        self.position_plot.set_title("Car's trajectory")
        self.position_plot.set_xlabel('position x (m)')
        self.position_plot.set_ylabel('position y (m)')

        self.position_plot.plot(odom_pos_x, odom_pos_y, color='r', label='Actual Trajectory')
        self.position_plot.plot(calc_pos_x, calc_pos_y,color='y' ,label="Calculated Position")
        self.position_plot.plot(pos_x, pos_y, color='g' ,label="Planned Trajectory")
        self.position_plot.legend(loc=2)

        plt.draw()
        self.mypause(0.0001)

    def position_listener_callback(self, msg):
        self.position_handler.handle_msg(msg)

    def trajectory_listener_callback(self, msg):
        self.trajectory_handler.handle_msg(msg)
    
    def reset_callback(self, request, response):
        print("Reseting plots")
        print(request)
        self.gps_handler.reset()
        self.imu_handler.reset()
        self.odom_handler.reset()
        self.control_handler.reset()
        self.position_handler.reset()
        self.trajectory_handler.reset()

        return response

        
def main(args=None):
    matplotlib.use("Qt5agg")
    rclpy.init(args=args)

    minimal_subscriber = StatSubscriber()

    plt.ion()
    plt.tight_layout()
    plt.show()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
