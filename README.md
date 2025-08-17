# PyBullet Multi-Robot Simulation

100台のロボットを10倍速でシミュレーションするPyBulletベースのプロジェクト

## 機能

- ✅ 100台のロボットの同時シミュレーション
- ✅ 10倍速シミュレーション
- ✅ URDFベースのロボット生成（アームロボット・移動ロボット）
- ✅ 接触判定機能
- ✅ 高度なURDFキャッシュシステム
- ✅ 外部データモニターウィンドウ
- ✅ パフォーマンス最適化（物理演算無効化対応）
- ✅ リアルタイムロボット制御（関節制御・速度制御）

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

## 実行方法

### 基本的な使用方法

```bash
# デフォルト: 10台のロボット、1倍速、30秒間
python configurable_demo.py

# 50台のロボット、5倍速で実行
python configurable_demo.py --robots 50 --speed 5

# 100台のロボット、10倍速、60秒間
python configurable_demo.py --robots 100 --speed 10 --duration 60

# 外部データモニターウィンドウ付きで実行
python configurable_demo.py --robots 20 --speed 2 --monitor

# GUI無効でパフォーマンス向上（200台、20倍速）
python configurable_demo.py --robots 200 --speed 20 --no-gui --verbose

# ヘルプを表示
python configurable_demo.py --help
```

### パフォーマンス最適化設定

```bash
# 最高パフォーマンス: GUI無効、物理演算無効、外部モニター
python configurable_demo.py --robots 100 --speed 10 --no-gui --monitor --verbose

# 高周波数シミュレーション（1000Hz、低速）
python configurable_demo.py --robots 10 --timestep 0.001 --speed 0.5
```

### パラメーター

**基本設定:**
- `--robots, -r`: ロボット台数（デフォルト: 10）
- `--speed, -s`: 実行速度倍率（デフォルト: 1.0）
- `--duration, -d`: シミュレーション時間（秒、デフォルト: 30）
- `--timestep, -t`: シミュレーション時間ステップ（秒、デフォルト: 0.004167、240Hz）

**表示・制御:**
- `--gui` / `--no-gui`: GUI有効/無効（デフォルト: 有効）
- `--physics` / `--no-physics`: 物理演算有効/無効（デフォルト: 無効）
- `--monitor`: 外部データモニターウィンドウを起動
- `--verbose, -v`: 詳細な出力を表示

## ファイル構成

- `configurable_demo.py` - **メインシミュレーション**（設定可能、URDF対応）
- `data_monitor.py` - 外部データモニターウィンドウ
- `robots/` - URDFロボット定義ファイル
  - `arm_robot.urdf` - 4自由度アームロボット
  - `mobile_robot.urdf` - 移動ロボット（前後移動・ヨー回転）
- `requirements.txt` - 必要なPythonパッケージ
- `memo.txt` - プロジェクトの仕様書（日本語）
- `CLAUDE.md` - プロジェクト詳細とコマンド一覧

## ロボット仕様

### アームロボット
- **固定ベース**: 床に固定（z=0）
- **制御方式**: 関節位置制御
- **自由度**: 4自由度（shoulder, elbow, wrist, end_effector）
- **動作**: ランダムな関節角度ターゲット

### 移動ロボット
- **移動範囲**: 地上高度0.3m
- **制御方式**: 直接速度制御
- **移動制限**: 前後移動とヨー回転のみ（横移動禁止）
- **動作**: ランダムな前後速度とヨー角速度

## パフォーマンス仕様

### 最適化技術
- **URDFキャッシュシステム**: ファイル内容、関節情報、視覚データの事前キャッシュ
- **バッチ処理**: 複数ロボットの同時生成
- **レンダリング制御**: 生成時の描画無効化
- **PyBulletエンジン最適化**: 非物理モード対応

### 達成性能
- **目標**: 100台ロボット @ 10倍速
- **生成速度**: 500+ robots/sec
- **実行速度**: 目標速度の維持
- **外部モニター**: パフォーマンス影響無し

## 外部データモニター

`--monitor`フラグで別ウィンドウでのリアルタイムデータ監視が可能：

- **表示データ**: シミュレーション時間、実時間、速度倍率、ロボット数、衝突数
- **更新頻度**: 0.5秒間隔
- **パフォーマンス**: メインシミュレーションに影響なし
- **GUI制御**: モニター有効時は画面上テキスト表示を自動無効化

## トラブルシューティング

### パフォーマンス問題
- GUI無効化: `--no-gui`
- 物理演算無効化: `--no-physics`（デフォルト）
- 外部モニター使用: `--monitor`（画面テキスト無効化）

### カメラ操作（GUIモード）
- **ズーム**: マウスホイール
- **回転**: 右クリック + ドラッグ  
- **移動**: 中クリック + ドラッグ

## 技術仕様

### 依存関係
- Python 3.8+
- PyBullet
- NumPy
- tkinter（外部モニター用）

### アーキテクチャ
- **オブジェクト指向設計**: ConfigurableRobotSimulation クラス
- **キャッシュシステム**: メモリ効率とパフォーマンス最適化
- **マルチプロセス対応**: 外部モニター（tkinter）
- **拡張性**: 新しいURDFロボット簡単追加