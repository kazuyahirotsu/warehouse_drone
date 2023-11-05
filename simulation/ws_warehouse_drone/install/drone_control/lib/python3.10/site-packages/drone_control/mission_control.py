import rclpy
from rclpy.node import Node
from .offboard_control import OffboardControl
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus
from std_srvs.srv import Trigger

class MissionControl(Node):
    def __init__(self):
        super().__init__('mission_control')
        self.offboard_control = OffboardControl()

        # Define waypoints as a list of tuples (x, y, z)
        self.waypoints = [
            (0.0, 0.0, -5.0),  # Takeoff and hover at 5m
            (10.0, 0.0, -5.0), # Move 10m to the east
            (10.0, 10.0, -5.0),# Move 10m to the north
            (0.0, 0.0, -5.0)   # Return to the starting position
        ]

        # Initialize the mission
        self.current_waypoint_index = 0  # Set to -1 to start with arming and taking off
        self.waypoint_threshold = 0.5  # How close the drone needs to be to the waypoint to be considered reached
        self.offboard_control_client = self.create_client(Trigger, 'activate_offboard')


        # Start the mission
        self.timer = self.create_timer(0.1, self.mission_callback)

    def mission_callback(self):

        self.offboard_control.publish_offboard_control_heartbeat_signal()

        # Check if offboard mode should be activated
        if self.offboard_control.offboard_setpoint_counter < 10:
            # Wait for the service to be available
            if not self.offboard_control_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().error('Offboard control service not available')
                return
            
            # Call the service to activate offboard mode
            request = Trigger.Request()
            future = self.offboard_control_client.call_async(request)
            future.add_done_callback(self.activate_offboard_callback)

            self.offboard_control.arm()

        # if self.offboard_control.offboard_setpoint_counter == 10:
        #     self.offboard_control.engage_offboard_mode()
        #     self.offboard_control.arm()

        if self.offboard_control.vehicle_local_position.z > self.offboard_control.takeoff_height and self.offboard_control.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.offboard_control.publish_position_setpoint(0.0, 0.0, self.offboard_control.takeoff_height)

        elif self.offboard_control.vehicle_local_position.z <= self.offboard_control.takeoff_height:
            self.offboard_control.land()
            exit(0)

        # if self.offboard_control.offboard_setpoint_counter < 11:
        #     self.offboard_control.offboard_setpoint_counter += 1

        self.get_logger().info(f'{self.offboard_control.vehicle_status.nav_state}')
        # self.get_logger().info(f'{self.offboard_control.vehicle_local_position}')
        # self.offboard_control.publish_offboard_control_heartbeat_signal()

        # if self.offboard_control.offboard_setpoint_counter == 10:
        #     # Arm the vehicle and switch to offboard mode
        #     self.offboard_control.engage_offboard_mode()
        #     self.offboard_control.arm()

        # if self.offboard_control.vehicle_local_position.z > self.offboard_control.takeoff_height and (0 <= self.current_waypoint_index < len(self.waypoints)) and self.offboard_control.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
        #     current_position = self.offboard_control.vehicle_local_position
        #     waypoint = self.waypoints[self.current_waypoint_index]

        #     # Check if we are close enough to the current waypoint to consider it reached
        #     if self.current_waypoint_index==0 or self.is_waypoint_reached(current_position, waypoint):
        #         self.get_logger().info(f'Waypoint {self.current_waypoint_index} reached: {waypoint}')
        #         self.current_waypoint_index += 1  # Move to the next waypoint
        #         if self.current_waypoint_index < len(self.waypoints):
        #             # Send the drone to the next waypoint
        #             next_waypoint = self.waypoints[self.current_waypoint_index]
        #             self.offboard_control.publish_position_setpoint(*next_waypoint)

        # elif self.current_waypoint_index == len(self.waypoints):
        #     self.get_logger().info('Mission completed')
        #     # Land the drone
        #     self.offboard_control.land()
        #     # Shutdown the node
        #     rclpy.shutdown()

        # if self.offboard_control.offboard_setpoint_counter < 11:
        #     self.offboard_control.offboard_setpoint_counter += 1

    def activate_offboard_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Offboard mode activated')
            else:
                self.get_logger().error(f'Failed to activate offboard mode: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Service call failed %r' % (e,))

    def is_waypoint_reached(self, vehicle_local_position, waypoint):
        # Extract the position from the VehicleLocalPosition object
        current_position = (vehicle_local_position.x, vehicle_local_position.y, vehicle_local_position.z)

        # Calculate the distance between the current position and the waypoint
        distance = sum([(c - w) ** 2 for c, w in zip(current_position, waypoint)]) ** 0.5
        return distance < self.waypoint_threshold


def main(args=None):
    rclpy.init(args=args)
    mission_control = MissionControl()
    rclpy.spin(mission_control)
    mission_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

