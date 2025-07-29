import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rai_core.rai.initialization.model_initialization import get_llm_model

class TurtleBot3Commander(Node):
    def __init__(self):
        super().__init__('turtlebot3_commander')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def send_cmd(self, linear=0.0, angular=0.0, duration=2.0):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.cmd_pub.publish(msg)
        self.get_logger().info(f"Sending cmd: linear={linear}, angular={angular}")
        time.sleep(duration)
        self.cmd_pub.publish(Twist())
        self.get_logger().info("Stopping.")

    # === LLM生成のジグザグ関数をここに貼り付ける ===

def main():
    rclpy.init()
    node = TurtleBot3Commander()
    llm = get_llm_model("complex_model")

    text = input("Enter a command (e.g., zigzag): ")
    prompt = (
        f"Convert the following user instruction into a sequence of robot commands "
        f"(zigzag only) "
        f"in JSON format like [\"zigzag\"]: {text}"
    )
    response = llm.invoke(prompt)
    content = getattr(response, "content", str(response))
    print(f"LLM response: {content}")

    if "zigzag" in content.lower():
        node.move_zigzag()

    rclpy.shutdown()

if __name__ == "__main__":
    main()
