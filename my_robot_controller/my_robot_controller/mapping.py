import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class TurtlebotMappingNode(Node):
    def __init__(self): 
        super().__init__("mapping_node") 
        self.get_logger().info("Mapping Node has started.")
        
        # Publisher to send movement commands 
        self._pose_publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        
        # Subscriber to receive LiDAR data 
        self._scan_listener = self.create_subscription(LaserScan, "/scan", self.robot_controller, 10)

        # Parameters to define obstacle detection width
        self._front_range_width = 5
        self._safe_distance = 1.0  # Threshold distance for obstacle detection

    def robot_controller(self, scan: LaserScan):
        cmd = Twist()
        
        # Extract directional distances for obstacle detection
        self._front = min(scan.ranges[:self._front_range_width + 1] + scan.ranges[-self._front_range_width:])
        self._left = min(scan.ranges[90 - self._front_range_width:90 + self._front_range_width + 1])
        self._right = min(scan.ranges[270 - self._front_range_width:270 + self._front_range_width + 1])
        
        # Log the sensor data for debugging purposes
        self.get_logger().info(f"Front: {self._front}, Left: {self._left}, Right: {self._right}")

        # If obstacle is detected in the front, avoid it
        if self._front < self._safe_distance:
            self.get_logger().info("Obstacle detected in front. Attempting to avoid...")
            
            # Turn based on which side is clearer
            if self._right < self._left:
                cmd.linear.x = 0.05  # Move slowly to give time for the robot to turn
                cmd.angular.z = 0.5  # Turn left
            else:
                cmd.linear.x = 0.05  # Move slowly to give time for the robot to turn
                cmd.angular.z = -0.5  # Turn right
        else:
            # No obstacle ahead, move forward
            cmd.linear.x = 0.3  # Move forward with moderate speed
            cmd.angular.z = 0.0  # Don't rotate
            
        # Publish the movement command
        self._pose_publisher.publish(cmd)
        
def main(args=None):
    rclpy.init(args=args)
    node = TurtlebotMappingNode()
    rclpy.spin(node)
    rclpy.shutdown()




