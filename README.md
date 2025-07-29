# 🚀 TurtleBot3 + LLM Demo

## 📝 概要
TurtleBot3 を **ROS 2 + Gazebo** シミュレーション環境で動作させ、  
**大規模言語モデル (LLM)** を用いて自然言語指示からロボットの動作を自動生成するデモです。

---

## 🎯 取り組み内容
- **自然言語 → ロボット動作コード生成**  
  GPT-4o を利用して、自然言語の指示を Python 関数へ変換。  
  その場で生成したコードを実行してロボットを動かします。

- **実装した動作例**
  - 前進・停止・回転
  - 斜め移動
  - ジグザグ移動
  - 複数動作の連続実行

---

## 📂 実装ファイル
- `turtlebot3_demo_diagonal.py` : 斜め移動デモ
- `turtlebot3_demo_diagonal_light.py` : 軽量版
- `turtlebot3_demo_diagonal_norai.py` : RAIライブラリ非依存版
- `turtlebot3_demo_multi.py` : 複数動作デモ
- `turtlebot3_demo_zigzag_llm.py` : LLM利用のジグザグ動作
- `turtlebot3_demo_codegen.py` : コード自動生成デモ
- `codegen_turtlebot3.py` : LLMによる動作関数自動生成
- `generated_action.py` : 自動生成された最新の動作関数

---

## 💡 技術的ポイント
- **LLM (GPT-4o)** を使用した自然言語からの Python コード生成
- **ROS 2 + Gazebo** によるシミュレーション
- コード生成 → 即時実行 のフローを実現

---

##  クレジット
- [RobotecAI/rai](https://github.com/RobotecAI/rai) を参照して実装を拡張

---
