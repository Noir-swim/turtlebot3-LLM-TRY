import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rai_core.rai.initialization.model_initialization import get_llm_model
import json, re

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

    def move_forward(self):
        self.send_cmd(linear=0.2, duration=2.0)

    def move_backward(self):
        self.send_cmd(linear=-0.2, duration=2.0)

    def turn_left(self):
        self.send_cmd(angular=0.5, duration=1.5)

    def turn_right(self):
        self.send_cmd(angular=-0.5, duration=1.5)

    def move_diagonal_forward_right(self):
        self.send_cmd(linear=0.2, angular=-0.2, duration=2.0)

        def move_zigzag(self):
        zigzag_duration = 1  # duration for each segment of the zigzag
        zigzag_angle = 0.5  # angular speed for turning
        straight_speed = 0.2  # linear speed for moving forward
        for _ in range(5):  # Adjust the range to control the number of zigzag segments
            self.send_cmd(straight_speed, zigzag_angle, zigzag_duration)
            self.send_cmd(straight_speed, -zigzag_angle, zigzag_duration)

def main():
    rclpy.init()
    node = TurtleBot3Commander()
    llm = get_llm_model("complex_model")

    text = input("Enter a command (e.g., zigzag): ")
    prompt = (
        f"Convert the following user instruction into a sequence of robot commands "
        f"(forward/backward/left/right/diagonal_forward_right/zigzag) "
        f"in JSON format like [\"zigzag\"]: {text}"
    )
    response = llm.invoke(prompt)
    content = getattr(response, "content", str(response))
    print(f"LLM response: {content}")

    match = re.search(r'\[.*\]', content)
    if match:
        try:
            actions = json.loads(match.group(0).replace("'", '"'))
        except json.JSONDecodeError:
            print("Failed to parse JSON from LLM response")
            actions = []
    else:
        actions = []

    for action in actions:
        a = action.lower()
        if a == "forward":
            node.move_forward()
        elif a == "backward":
            node.move_backward()
        elif a == "left":
            node.turn_left()
        elif a == "right":
            node.turn_right()
        elif a == "diagonal_forward_right":
            node.move_diagonal_forward_right()
        elif a == "zigzag":
            node.move_zigzag()
        else:
            print(f"Unknown action: {action}")

    rclpy.shutdown()

if __name__ == "__main__":
    main()
