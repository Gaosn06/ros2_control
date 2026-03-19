



import rclpy
from rclpy.node import Node
from control_msgs.msg import JointJog

class PublishCommand (Node):
    def __init__(self,):
        super().__init__("publish_command")
        self.command_publisher_ = self.create_publisher(JointJog, "my_controller/reference", 10)
        self.timer_             = self.create_timer(3, self.publish_command_callback)
        self.msg = JointJog()
        self.msg.joint_names = ["joint1", "joint2"]
        self.datas = [[0.0,0.0],[-1.0,-1.0],[0.0,0.0],[1.0,1.0]]
        self.v_datas = [[0.2,0.7],[-1.0,-1.0],[0.0,0.0],[1.0,1.0]]
        self.i = 0

    def publish_command_callback(self):
        self.msg.displacements = self.datas[self.i % 4]
        self.msg.velocities    = self.v_datas[0]
        self.i += 1
        self.command_publisher_.publish(self.msg)

def main(args=None):
    rclpy.init(args=args)
    node = PublishCommand()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()




