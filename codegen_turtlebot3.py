import importlib.util
import os
import time
import openai
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

openai.api_key = os.getenv("OPENAI_API_KEY")
GENERATED_FILE = "generated_action.py"

class TurtleBot3Commander(Node):
    def __init__(self):
        super().__init__('turtlebot3_commander')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

    def send_cmd(self, linear, angular, duration):
        twist = Twist()
        twist.linear.x = float(linear)
        twist.angular.z = float(angular)
        self.publisher.publish(twist)
        self.get_logger().info(f"Sending cmd: linear={linear}, angular={angular}")
        time.sleep(duration)
        # Stop after each movement
        stop = Twist()
        self.publisher.publish(stop)

def save_and_import_function(code):
    with open(GENERATED_FILE, "w") as f:
        f.write(code + "\n")
    spec = importlib.util.spec_from_file_location("generated_action", GENERATED_FILE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return getattr(module, "move_custom")

def main():
    rclpy.init()
    node = TurtleBot3Commander()

    while True:
        prompt = input("Enter movement description (or 'run' to execute last): ")
        if prompt.lower() == "run":
            from generated_action import move_custom
            move_custom(node)
            continue

        print("=== Requesting code from LLM ===")
        response = openai.ChatCompletion.create(
            model="gpt-4o",
            messages=[
                {"role": "system", "content": "Generate only Python code."},
                {"role": "user", "content": (
                    "Output only one Python code block.\n"
                    "Define exactly one top-level function: def move_custom(self):\n"
                    "Inside, call self.send_cmd(linear, angular, duration) at least twice.\n"
                    f"Implement this behavior: {prompt}\n"
                    "Do not output explanations or text outside the code block."
                )}
            ],
            max_tokens=300
        )
        code = response.choices[0].message["content"].strip()
        if code.startswith("```"):
            code = "\n".join(code.split("\n")[1:-1])
        print("\n=== Generated Code ===")
        print(code)
        save_and_import_function(code)
        print(f"Code saved to {GENERATED_FILE}")

    rclpy.shutdown()

if __name__ == "__main__":
    main()
