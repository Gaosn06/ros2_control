import rclpy
from rclpy.node import Node
from control_msgs.msg import JointJog
from controller_communicate_type_pkg.msg import JointStates
class RecState(Node):
    def __init__(self):
        super().__init__("rec_state")
        self.subscription = self.create_subscription(JointStates, "my_controller/state", self.listener_callback, 10)

    def listener_callback(self, msg):
        self.get_logger().info(f"time: {msg.header.stamp.sec}.{msg.header.stamp.nanosec}")
        for i in range(0,2):
            self.get_logger().info(f"{msg.joint_names[i]}/displacements:{msg.displacements[i]}")
            self.get_logger().info(f"{msg.joint_names[i]}/velocities:{msg.velocities[i]}")
            self.get_logger().info(f"{msg.joint_names[i]}/efforts:{msg.efforts[i]}")
            self.get_logger().info(f"{msg.sensor_names[i]}/torques:{msg.torques[i]}")
        self.get_logger().info("---------------------------------------------")

def main(args=None):
    rclpy.init(args=args)
    node = RecState()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()



