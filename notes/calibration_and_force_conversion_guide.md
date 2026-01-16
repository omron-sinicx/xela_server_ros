# XELA センサー キャリブレーション・力変換 使用ガイド

このドキュメントでは、XELAセンサーのデジタル値から力への変換機能の使い方を説明します。

## 1. キャリブレーションスクリプト

### 概要

`calibrate_digital_to_force.py` は、直交距離回帰 (ODR) を使用してデジタル値から力への変換パラメータを算出します。

### 必要なファイル

キャリブレーションデータディレクトリに以下のCSVファイルが必要です：

- `tax_x.csv` - X軸のキャリブレーションデータ
- `tax_y.csv` - Y軸のキャリブレーションデータ
- `tax_z.csv` - Z軸のキャリブレーションデータ

各CSVファイルのフォーマット：

```csv
digital,measured_frc
523330,4.42
522867,3.84
...
```

### コマンド例

```bash
# 特定のディレクトリを指定してキャリブレーション実行
python3 calibrate_digital_to_force.py --data-dir data/xr1944-2110-45-ctrlr-4

# 絶対パスで指定
python3 calibrate_digital_to_force.py --data-dir /path/to/calibration/data

# デフォルトパス (data/digital_to_force) を使用
python3 calibrate_digital_to_force.py
```

### 出力

指定したディレクトリに以下のファイルが生成されます：

- `calibration_params.json` - キャリブレーションパラメータ
- `calibration_plot.png` - 回帰結果のプロット

---

## 2. 力変換ノード

### 概要

`xela_force_converter.py` は、XELAセンサーからのデジタル値をキャリブレーションパラメータを使用して力に変換し、ROSトピックとして出版します。

### 出版されるトピック

各センサーごとに以下のトピックが出版されます：

- `/xela_force_converter/sensor_<n>/forces` (Float32MultiArray) - 各タクセルの力 (16x3)
- `/xela_force_converter/sensor_<n>/total_force` (Vector3Stamped) - 全タクセル合計力 (1x3)

### コマンド例

```bash
# 基本的な起動（デフォルトキャリブレーション使用）
roslaunch xela_server_ros force_converter.launch

# センサー1にキャリブレーションJSONを指定
roslaunch xela_server_ros force_converter.launch \
    sensor_1_calib:=data/xr1944-2110-45-ctrlr-4/calibration_params.json

# センサー1とセンサー2に異なるキャリブレーションを指定
roslaunch xela_server_ros force_converter.launch \
    sensor_1_calib:=data/sensor1_calib/calibration_params.json \
    sensor_2_calib:=data/sensor2_calib/calibration_params.json

# デフォルトキャリブレーション（特定センサー用がない場合に使用）
roslaunch xela_server_ros force_converter.launch \
    default_calib:=data/xr1944-2110-45-ctrlr-4/calibration_params.json
```

### パラメータ

| パラメータ | 説明 |
|-----------|------|
| `sensor_1_calib` | センサー1用のキャリブレーションJSONファイルパス |
| `sensor_2_calib` | センサー2用のキャリブレーションJSONファイルパス |
| `default_calib` | デフォルトのキャリブレーションJSONファイルパス（未指定センサー用） |

パスはパッケージからの相対パスまたは絶対パスで指定可能です。

---

## 3. 典型的なワークフロー

### Step 1: キャリブレーションデータの準備

```bash
# 新しいセンサー用のディレクトリを作成
mkdir -p data/my_sensor_calib

# tax_x.csv, tax_y.csv, tax_z.csv をコピー
cp /path/to/source/*.csv data/my_sensor_calib/
```

### Step 2: キャリブレーション実行

```bash
python3 scripts/calibrate_digital_to_force.py --data-dir data/my_sensor_calib
```

### Step 3: 力変換ノードを起動

```bash
roslaunch xela_server_ros force_converter.launch \
    sensor_1_calib:=data/my_sensor_calib/calibration_params.json
```

### Step 4: 出力確認

```bash
rostopic echo /xela_force_converter/sensor_1/total_force
```

---

## 4. キャリブレーションパラメータの構造

`calibration_params.json` の構造：

```json
{
  "total": {
    "x": { "slope": 0.00069, "intercept": -357.9, ... },
    "y": { "slope": 0.00056, "intercept": -296.5, ... },
    "z": { "slope": 0.00085, "intercept": -508.9, ... }
  },
  "each": {
    "x": { "slope": 0.00069, "intercept": -22.37, ... },
    "y": { "slope": 0.00056, "intercept": -18.53, ... },
    "z": { "slope": 0.00085, "intercept": -31.80, ... }
  }
}
```

- `total`: センサー全体（16タクセル合計）のパラメータ
- `each`: 各タクセル1個あたりのパラメータ（interceptのみ16で割った値）

力変換には `each` セクションのパラメータが使用されます。
