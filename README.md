# 🤖 TurtleBot3 × LLM Demo

ROS 2 + Gazebo 環境上で **自然言語で TurtleBot3 を操作** することに挑戦したデモプロジェクトです。  
OpenAI GPT を活用して、人間の言葉からロボットの動作コードを生成・実行する仕組みを実装しました。

---

## ✨ 取り組み内容
- **TurtleBot3 のシミュレーション**  
  Gazebo 上でのロボット動作環境構築
- **自然言語処理 (GPT-4o)**  
  「前に進む」「右に曲がって前に進む」など自然な日本語コマンドを理解
- **動作スクリプト自動生成**  
  LLM が Python コードを生成し、その場でロボットを動作させる
- **複雑動作への挑戦**  
  ジグザグ走行・回転・停止などの連続動作を一回の指示で実行

---

## 📂 構成ファイル
- `codegen_turtlebot3.py`  
  LLM に指示して動作コードを生成・実行するメインスクリプト
- `turtlebot3_demo_*.py`  
  各種試作スクリプト  
  - `*_diagonal*.py` … 斜め移動デモ  
  - `*_multi.py` … 複数動作デモ  
  - `*_zigzag_*.py` … ジグザグ移動試作

---

## 🚀 使い方（概要）
```bash
# Gazebo で TurtleBot3 を起動
ros2 launch turtlebot3_gazebo turtlebot3_empty_world.launch.py

# コード生成スクリプトを実行
python codegen_turtlebot3.py

# LLM に動作指示を送信
Enter movement description (or 'run' to execute last):
「前に進んでから右に曲がる」

---

## 📕クレジット
- [RobotecAI/rai](https://github.com/RobotecAI/rai) を参照して実装を拡張。
