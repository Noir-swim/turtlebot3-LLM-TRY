# TurtleBot3 + LLM Demo

このリポジトリは、LLM (gpt-4o) を用いて自然言語から TurtleBot3 の動作コードを自動生成するデモです。

## 特徴
- LLMに指示を与えると、Python関数 `move_custom()` が自動生成されます。
- 生成されたコードを即時に実行し、Gazebo上のTurtleBot3を動作させます。

## 実行ファイル
- `codegen_turtlebot3.py` : LLMへの指示入力 & コード生成・実行
- `generated_action.py` : 生成された動作コードのサンプル

## クレジット
- [RobotecAI/rai](https://github.com/RobotecAI/rai) を参照して実装を拡張。
