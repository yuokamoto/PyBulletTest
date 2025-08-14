# PyBullet Multi-Robot Simulation

100台のロボットを10倍速でシミュレーションするPyBulletベースのプロジェクト

## 機能

- ✅ 100台のロボットの同時シミュレーション
- ✅ 10倍速シミュレーション
- ✅ 接触判定機能
- ✅ ランダムな動作制御
- ✅ 物理演算の最適化（パフォーマンス重視）

## セットアップ

### 仮想環境の構築（推奨）

プロジェクトの依存関係を分離するため、venv仮想環境の使用を推奨します。

```bash
# 1. 仮想環境を作成
python -m venv venv

# 2. 仮想環境を有効化
# Linux/Mac の場合:
source venv/bin/activate

# Windows の場合:
# venv\Scripts\activate

# 3. 必要なパッケージをインストール
pip install -r requirements.txt
```

### 仮想環境の無効化

作業が終わったら仮想環境を無効化できます：

```bash
deactivate
```

### 直接インストール（非推奨）

仮想環境を使わない場合は以下のコマンドでも実行可能ですが、システムPythonに影響する可能性があります：

```bash
pip install -r requirements.txt
```

## 実行方法

### 設定可能なデモ（推奨）
台数とrealtime factorを自由に設定できるデモ：

```bash
# デフォルト: 10台のロボット、1倍速、30秒間
python configurable_demo.py

# 50台のロボット、5倍速で実行
python configurable_demo.py --robots 50 --speed 5

# 100台のロボット、10倍速、60秒間
python configurable_demo.py --robots 100 --speed 10 --duration 60

# 高周波数シミュレーション（1000Hz、低速）
python configurable_demo.py --robots 10 --timestep 0.001 --speed 0.5

# GUI無効でパフォーマンス向上（200台、20倍速、低周波数）
python configurable_demo.py --robots 200 --speed 20 --timestep 0.008 --no-gui --verbose

# ヘルプを表示
python configurable_demo.py --help
```

**パラメーター:**
- `--robots, -r`: ロボット台数（デフォルト: 10）
- `--speed, -s`: 実行速度倍率（デフォルト: 1.0）
- `--timestep, -t`: シミュレーション時間ステップ（秒、デフォルト: 0.004167、240Hz）
- `--duration, -d`: シミュレーション時間（秒、デフォルト: 30）
- `--no-gui`: GUI無効化（パフォーマンス向上）
- `--verbose, -v`: 詳細な出力を表示

### メインシミュレーション（100台のロボット、固定設定）
```bash
python multi_robot_simulation.py
```

### シンプルなテスト（10台のロボット）
```bash
python simple_example.py
```

## ファイル構成

- `configurable_demo.py` - **設定可能なデモ**（台数・速度をパラメーター指定可能）
- `multi_robot_simulation.py` - メインの100台ロボットシミュレーション
- `simple_example.py` - 10台のロボットによるテスト用シミュレーション
- `requirements.txt` - 必要なPythonパッケージ
- `memo.txt` - プロジェクトの仕様書（日本語）

## シミュレーション仕様

### 実装済み機能
- **ロボット生成**: シンプルなボックス形状のロボット
- **ランダム移動**: 各ロボットがランダムな方向に移動
- **接触判定**: ロボット間の衝突検出
- **高速シミュレーション**: 10倍速での実行
- **スケーラブル設計**: 100台のロボットを効率的にシミュレーション

### 今後の拡張予定
- URDF からのロボット生成
- ROS 2 との統合
- より複雑な制御インターフェース
- simulation_interfaces対応