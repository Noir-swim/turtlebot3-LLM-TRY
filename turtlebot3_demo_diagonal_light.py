import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rai_core.rai.initialization.model_initialization import get_llm_model

class TurtleBot3Commander(Node):
    def __init__(self):
        super().__init__('turtlebot3_commander')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

    def send_cmd(self, linear=0.0, angular=0.0, duration=1.0):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.publisher_.publish(twist)
        self.get_logger().info(f"Sending cmd: linear={linear}, angular={angular}")
        self.create_timer(duration, lambda: self.stop(), oneshot=True)

    def stop(self):
        twist = Twist()
        self.publisher_.publish(twist)
        self.get_logger().info("Stopping.")

    # 新しい動作
    def move_forward(self):
        self.send_cmd(linear=0.2, duration=2.0)

    def turn_right(self):
        self.send_cmd(angular=-0.5, duration=1.5)

    def turn_left(self):
        self.send_cmd(angular=0.5, duration=1.5)

    def move_diagonal_forward_right(self):
        self.send_cmd(linear=0.2, angular=-0.2, duration=2.0)

def main():
    rclpy.init()
    node = TurtleBot3Commander()
    llm = get_llm_model("complex_model")

    while rclpy.ok():
        text = input("Enter a command (自然言語): ")
        response = llm.invoke(text)
        print(f"LLM response: {response}")

        cmd = response.content.lower()
        if "diagonal" in cmd:
            node.move_diagonal_forward_right()
        elif "forward" in cmd:
            node.move_forward()
        elif "right" in cmd:
            node.turn_right()
        elif "left" in cmd:
            node.turn_left()
        else:
            node.stop()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
