import importlib.util
import os
import openai

# OpenAI APIキーを環境変数から取得
openai.api_key = os.getenv("OPENAI_API_KEY")

GENERATED_FILE = "generated_action.py"

def save_and_import_function(code):
    with open(GENERATED_FILE, "w") as f:
        f.write(code + "\n")

    spec = importlib.util.spec_from_file_location("generated_action", GENERATED_FILE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return getattr(module, "move_custom")

def main():
    while True:
        prompt = input("Enter new action description (or 'run' to execute last): ")
        if prompt.strip().lower() == "run":
            from generated_action import move_custom
            class DummyRobot:
                def send_cmd(self, linear, angular, duration):
                    print(f"[DummyRobot] send_cmd -> linear={linear}, angular={angular}, duration={duration}")
            node = DummyRobot()
            move_custom(node)
            continue

        print("=== Requesting code from LLM ===")
        response = openai.ChatCompletion.create(
            model="gpt-3.5-turbo",
            messages=[
                {"role": "system", "content": "You are a Python code generator."},
                {"role": "user", "content": prompt}
            ],
            max_tokens=300
        )
        code = response.choices[0].message["content"].strip()
        if code.startswith("```"):
            code = "\n".join(code.split("\n")[1:-1])
        print("\n=== Generated Code ===")
        print(code)
        save_and_import_function(code)
        print("Code saved as", GENERATED_FILE)

if __name__ == "__main__":
    main()
