import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float64


class InverseKinematics(Node):
    def __init__(self):
        super().__init__("inverse_kinematics")

        # Must match wheel_radius and wheel separation in the robot URDF.
        self.wheel_radius = 0.1
        self.wheel_separation = 0.45

        self.create_subscription(Twist, "/input_ik", self.velocity_callback, 10)
        self.left_publisher = self.create_publisher(Float64, "/left_wheel/command", 10)
        self.right_publisher = self.create_publisher(
            Float64, "/right_wheel/command", 10
        )

        self.get_logger().info("Inverse kinematics aktif.")

    def velocity_callback(self, msg):
        v = msg.linear.x  # m/s
        w = msg.angular.z  # rad/s

        # Kecepatan linear tiap roda (m/s)
        v_left = v - (w * self.wheel_separation / 2.0)
        v_right = v + (w * self.wheel_separation / 2.0)

        # Konversi ke kecepatan sudut roda (rad/s)
        left_msg = Float64()
        right_msg = Float64()
        left_msg.data = v_left / self.wheel_radius
        right_msg.data = v_right / self.wheel_radius

        self.left_publisher.publish(left_msg)
        self.right_publisher.publish(right_msg)


def main(args=None):
    rclpy.init(args=args)
    node = InverseKinematics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
