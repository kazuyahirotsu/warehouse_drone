import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus

import sys
import threading
import geometry_msgs.msg
if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty

class KeyboardControl(Node):
    """Node for controlling a vehicle in offboard mode."""

    def __init__(self) -> None:
        super().__init__('offboard_control_takeoff_and_land')

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # Create subscribers
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)

        # Initialize variables
        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.takeoff_height = -1.0

        # Initialize the mission
        self.current_waypoint_index = 0  # Set to -1 to start with arming and taking off
        self.waypoint_threshold = 0.5  # How close the drone needs to be to the waypoint to be considered reached

        # # Create a timer to publish control commands
        self.timer = self.create_timer(0.1, self.timer_callback_keyboard)

        self.settings = self.saveTerminalSettings()
        self.target_position = [0.0,0.0,0.0]
        self.target_yaw = 0.0

        self.key = ''
        # Create a thread for keyboard input handling
        self.keyboard_thread = threading.Thread(target=self.keyboard_input_thread)
        self.keyboard_thread.daemon = True  # Make the thread a daemon so it terminates when the main program exits
        self.keyboard_thread.start()

    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_local_position = vehicle_local_position
        # Print out the current velocity and acceleration (if available)
        # self.get_logger().info(f"Current velocity: vx: {vehicle_local_position.vx}, vy: {vehicle_local_position.vy}, vz: {vehicle_local_position.vz}")
        # if hasattr(vehicle_local_position, 'ax') and hasattr(vehicle_local_position, 'ay') and hasattr(vehicle_local_position, 'az'):
            # self.get_logger().info(f"Current acceleration: ax: {vehicle_local_position.ax}, ay: {vehicle_local_position.ay}, az: {vehicle_local_position.az}")


    def vehicle_status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = vehicle_status

    # def keyboard_callback(self, msg: Key):
    #     self.keyboard_status = msg

    def arm(self):
        """Send an arm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def disarm(self):
        """Send a disarm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')

    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")

    def land(self):
        """Switch to land mode."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Switching to land mode")

    def publish_offboard_control_heartbeat_signal(self):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_position_setpoint(self, x: float, y: float, z: float):
        """Publish the trajectory setpoint."""
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = 1.57079  # (90 degree)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        self.get_logger().info(f"Publishing position setpoints {[x, y, z]}")

    def publish_vehicle_command(self, command, **params) -> None:
        """Publish a vehicle command."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def is_waypoint_reached(self, vehicle_local_position, waypoint):
        # Extract the position from the VehicleLocalPosition object
        current_position = (vehicle_local_position.x, vehicle_local_position.y, vehicle_local_position.z)

        # Calculate the distance between the current position and the waypoint
        distance = sum([(c - w) ** 2 for c, w in zip(current_position, waypoint)]) ** 0.5
        return distance < self.waypoint_threshold

    def timer_callback(self) -> None:
        """Callback function for the timer."""
        self.publish_offboard_control_heartbeat_signal()
        current_position = self.vehicle_local_position

        if self.offboard_setpoint_counter == 10:
            self.engage_offboard_mode()
            self.arm()

        if self.current_waypoint_index < len(self.waypoints) < self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            current_position = self.vehicle_local_position
            waypoint = self.waypoints[self.current_waypoint_index]

            # Check if we are close enough to the current waypoint to consider it reached
            if self.is_waypoint_reached(current_position, waypoint):
                self.get_logger().info(f'Waypoint {self.current_waypoint_index} reached: {waypoint}')
                self.current_waypoint_index += 1  # Move to the next waypoint

                if self.current_waypoint_index < len(self.waypoints):
                    # Send the drone to the next waypoint
                    waypoint = self.waypoints[self.current_waypoint_index]
                    self.publish_position_setpoint(*waypoint)
            else:
                self.publish_position_setpoint(*waypoint)

        elif self.current_waypoint_index == len(self.waypoints) :
            self.land()
            exit(0)

        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1

    def getKey(self, settings):
        if sys.platform == 'win32':
            # getwch() returns a string on Windows
            key = msvcrt.getwch()
        else:
            tty.setraw(sys.stdin.fileno())
            # sys.stdin.read() returns a string on Linux
            key = sys.stdin.read(1)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key


    def saveTerminalSettings(self):
        if sys.platform == 'win32':
            return None
        return termios.tcgetattr(sys.stdin)
    
    def restoreTerminalSettings(self, old_settings):
        if sys.platform == 'win32':
            return
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


    def keyboard_input_thread(self):
        while True:
            self.key = self.getKey(self.settings)
            self.get_logger().info(f'Keyboard input: {self.key}')

            if self.key == 'q':  # ASCII for 'q'
                self.disarm()
            # Handle other keyboard inputs here

            # Add a sleep to control the rate of keyboard input processing (adjust as needed)
            # rclpy.sleep(0.1)

    def timer_callback_keyboard(self):
        """Method to control the drone with keyboard inputs."""
            
        self.publish_offboard_control_heartbeat_signal()
        current_position = self.vehicle_local_position

        self.get_logger().info(f'{self.key}')

        if self.offboard_setpoint_counter == 10:
            self.engage_offboard_mode()
            self.arm()

        if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            current_position = self.vehicle_local_position

            # Keycode for 'up' arrow
            if self.key == 'i':  # ASCII for 'w'
                self.target_position[1] += 1
            # Keycode for 'down' arrow
            if self.key == 'k':  # ASCII for 's'
                self.target_position[1] -= 1
            # Keycode for 'left' arrow
            if self.key == 'j':  # ASCII for 'a'
                self.target_position[0] -= 1
            # Keycode for 'right' arrow
            if self.key == 'l':  # ASCII for 'd'
                self.target_position[0] += 1
            # Keycode for increasing altitude ('w')
            if self.key == 'w':  # ASCII for 't'
                self.target_position[2] -= 1
            # Keycode for decreasing altitude ('s')
            if self.key == 's':  # ASCII for 'b'
                self.target_position[2] += 1
            # Keycode for rotating counter-clockwise
            if self.key == 'a':  # ASCII for 'j'
                self.target_yaw -= 0.1
            # Keycode for rotating clockwise
            if self.key == 'd':  # ASCII for 'k'
                self.target_yaw += 0.1

            self.get_logger().info(f"target: {self.target_position}, yaw: {self.target_yaw}")

            self.publish_position_setpoint_with_yaw(self.target_position[0], self.target_position[1], self.target_position[2], self.target_yaw)

        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1

    def publish_position_setpoint_with_yaw(self, x: float, y: float, z: float, yaw: float):
        """Publish the trajectory setpoint with yaw."""
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = yaw
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        self.get_logger().info(f"Publishing position setpoints {[x, y, z]} with yaw {yaw}")


def main(args=None) -> None:
    print('Starting offboard control node...')
    
    rclpy.init(args=args)
    offboard_control = KeyboardControl()

    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
