# XELA Sensor ROS パッケージ セットアップガイド

このドキュメントは、Docker環境内でXELAセンサーをROS 1（Noetic/ROS One）で使用するためのセットアップ手順と知見をまとめたものです。

## 目次

1. [パッケージ構成](#パッケージ構成)
2. [AppImageの展開](#appimageの展開)
3. [設定ファイル (xServ.ini)](#設定ファイル-xservini)
4. [CANバスの設定](#canバスの設定)
5. [起動方法](#起動方法)
6. [WebSocketデータ構造](#websocketデータ構造)
7. [モジュール詳細](#モジュール詳細)
8. [トラブルシューティング](#トラブルシューティング)

---

## パッケージ構成

```
xela_server_ros/
├── software/               # XELA ソフトウェア（.gitignore）
│   └── v1.7.7/             # バージョンディレクトリ
│       ├── server/         # xela_server展開後のファイル
│       │   └── AppRun      # メインエントリポイント
│       ├── conf/           # xela_conf展開後のファイル
│       │   └── AppRun
│       ├── log/            # xela_log展開後のファイル
│       │   └── AppRun
│       ├── viz/            # xela_viz展開後のファイル
│       │   └── AppRun
│       └── config/         # 設定ファイル
│           └── xServ.ini   # センサー設定
├── scripts/
│   ├── xela_server         # ROSノード用ラッパースクリプト（bash）
│   ├── xela_service        # ROSサービス/トピックノード（Python）
│   └── xela_cluster_monitor.py  # クラスタリング監視ツール
├── launch/
│   └── service.launch      # roslaunch用ファイル
├── msg/                    # ROSメッセージ定義
├── srv/                    # ROSサービス定義
├── notes/                  # ドキュメント
└── .gitignore              # software/ を無視
```

### 重要なポイント

- `software/` は `.gitignore` で無視されている（環境依存のため）
- AppImageは Docker 内で FUSE が使えないため、展開して使用する必要がある

---

## AppImageの展開

### 手順

1. AppImage ファイルを取得（XELA から提供）

2. 展開:

   ```bash
   cd ~/osx-ur/catkin_ws/src/xela_server_ros
   chmod +x xela_server-1.7.7.AppImage
   ./xela_server-1.7.7.AppImage --appimage-extract
   ```

3. ファイルを配置:

   ```bash
   # software/v1.7.7/bin/server/ ディレクトリに配置
   mkdir -p software/v1.7.7/bin/server
   mv squashfs-root/* software/v1.7.7/bin/server/

   # config/ に設定ファイルを配置
   mkdir -p software/v1.7.7/config
   cp software/v1.7.7/bin/server/etc/xela/xServ.ini software/v1.7.7/config/

   # ツールの展開と配置
   ./xela_conf --appimage-extract && mv squashfs-root software/v1.7.7/bin/conf
   ./xela_log --appimage-extract && mv squashfs-root software/v1.7.7/bin/log
   ./xela_viz --appimage-extract && mv squashfs-root software/v1.7.7/bin/viz
   ```

4. 実行確認:

   ```bash
   ./software/v1.7.7/bin/server/AppRun --help
   ```

---

## 設定ファイル (xServ.ini)

### 基本構成

```ini
[server]
bustype = socketcan
channel = can0

[viz]
max_offset = 100
max_size = 100
arrows = off
grid = on
transparency = off
origins = off
fps = off
ups = off
temp_show = on

[debug]
sens_print = full

[sensor]
num_of_sensor = 2
model = uSPa44
id = 4
calibration = on
clustering = on

[sensor2]
id = 6
```

### 設定パラメータ詳細

#### [server] セクション

| パラメータ | 値 | 説明 |
|-----------|-----|------|
| `bustype` | `socketcan`, `pcan`, `sim` | CANバスのタイプ。Linuxでは通常 `socketcan` |
| `channel` | `can0`, `slcan0` 等 | CANインターフェース名 |

#### [sensor] セクション

| パラメータ | 値 | 説明 |
|-----------|-----|------|
| `num_of_sensor` | 整数 | センサー数 |
| `model` | `uSPa44`, `XR1944` 等 | センサーモデル名 |
| `id` | 0-15 (10進数) | コントローラーID（CAN IDの一部） |
| `calibration` | `on`/`off` | 力キャリブレーション有効化 |
| `clustering` | `on`/`off` | クラスタリング有効化 |
| `magcomp` | `on`/`off` | 磁気補正（対応センサーのみ） |
| `channel` | 0, 1, 2... | I2Cチャンネル（SDA）|
| `rotation` | 0-3 | 回転（0°, 90°, 180°, 270°）|

### センサーモデルの選択

| CAN IDパターン | 使用するモデル |
|---------------|---------------|
| 0X4, 0X4+40, 1X4, 1X4+40... | `XR1944` |
| 0X4, 1X4, 2X4, 3X4... | `uSPa44` ← **多くの場合こちら** |

**重要**: センサーのラベルが「XR1944」でも、CAN IDフォーマットによっては `uSPa44` を設定する必要がある。`candump can0` でCAN IDを確認し、適切なモデルを選択する。

### 複数センサーの設定

2つ目以降のセンサーは `[sensorN]` セクションで個別設定:

```ini
[sensor]
num_of_sensor = 2
model = uSPa44
id = 4

[sensor2]
id = 6
```

---

## CANバスの設定

### socketcan のセットアップ（Docker コンテナ内）

```bash
# ビットレート 1Mbps で can0 を設定
sudo ip link set can0 type can bitrate 1000000
sudo ip link set up can0

# 確認
ip link show can0
```

### CAN トラフィックの確認

```bash
# can-utils のインストール
apt update && apt install -y can-utils

# トラフィック確認
candump can0

# 出力例:
#  can0  004   [8]  93 7E 31 7E 15 93 12 0F
#  can0  104   [8]  93 84 06 80 55 91 BB 0C
```

### CAN ID の読み方

```
XY4 形式の場合（uSPa44）:
  X = 行インデックス (0-3)
  Y = 列インデックス (0-3)
  4 = センサーID（10進数の4 = 0x04の末尾）

例: 134 = 行1, 列3, センサーID 4
```

---

## 起動方法

### 方法1: roslaunch（推奨）

```bash
source ~/osx-ur/catkin_ws/devel/setup.bash
roslaunch xela_server_ros service.launch
```

### 方法2: 個別起動

**ターミナル1 - サーバー:**

```bash
cd ~/osx-ur/catkin_ws/src/xela_server_ros
./software/v1.7.7/bin/server/AppRun -f software/v1.7.7/config/xServ.ini --port 5000 --ip 192.168.0.21
```

**ターミナル2 - ROSノード:**

```bash
source ~/osx-ur/catkin_ws/devel/setup.bash
rosrun xela_server_ros xela_service --ip 192.168.0.21 --port 5000
```

### 方法3: クラスタリング監視

```bash
rosrun xela_server_ros xela_cluster_monitor.py --ip 192.168.0.21 --port 5000
```

---

## WebSocketデータ構造

`xela_server` は WebSocket 経由でJSONデータを送信する。

### メッセージ例

```json
{
  "message": 983,
  "time": 1768296808.56,
  "sensors": 2,
  "type": "welcome",
  "1": {
    "time": 1768296808.55,
    "sensor": "1",
    "data": "8197,829A,91E6,...",
    "model": "uSPa44",
    "taxels": 16,
    "calibrated": [0.003, -0.002, 0.017, ...],
    "extradots": [["1.0", "1.42", "21.62"]],
    "special": [[33175, 33434, 37350, 0.0, ...], ...],
    "temp": [0.0, 0.0, ...],
    "ups": [105.2, 105.3, ...]
  },
  "2": { ... }
}
```

### フィールド説明

| フィールド | 型 | 説明 |
|-----------|-----|------|
| `data` | string/list | 生の磁気データ（16進数文字列またはリスト） |
| `calibrated` | list[float] | キャリブレーション済み力データ（48値 = 16タクセル × 3軸） |
| `extradots` | list | クラスタリング結果 `[[X座標, Y座標, 力], ...]` |
| `special` | list | タクセル拡張データ（各タクセル12値） |
| `temp` | list[float] | 温度データ（16値） |
| `ups` | list[float] | タクセル更新レート（Hz） |

### ROSトピック

| トピック | メッセージ型 | 説明 |
|---------|-------------|------|
| `/xServTopic` | `SensStream` | センサーデータストリーム |
| `/xServStream` | サービス | オンデマンドデータ取得 |

---

## モジュール詳細

### Force Calibration（力キャリブレーション）

```ini
calibration = on
```

- 生の磁気データを物理的な力（ニュートン）に変換
- `calibrated` フィールドに出力される（48値 = 16タクセル × X/Y/Z）
- ROSメッセージの `forces` フィールドに対応

### Clustering（クラスタリング）

```ini
clustering = on
```

- 複数タクセルへの接触を1つの「タッチポイント」として計算
- `extradots` フィールドに出力される
- 各クラスター: `[X座標, Y座標, 力/強度]`
- X/Y座標はタクセル間で補間された値（0.0〜3.0）

### Magnetic Compensation（磁気補正）

```ini
magcomp = on
```

- **対応センサーのみ**で有効
- 非対応センサーで有効化すると無効なデータが出力される
- `rawmode = on` で補正前後の両方のデータを取得可能

---

## トラブルシューティング

### "Address already in use" エラー

```bash
# ポート5000を使用しているプロセスを終了
pkill -9 -f xela_server
pkill -9 -f AppRun
pkill -9 -f roslaunch

# 確認
ss -tlnp | grep 5000
```

### "Connection refused" エラー

- `xela_server` がまだ起動完了していない（起動に数秒かかる）
- 数回リトライすれば自動的に接続される

### すべてのタクセルがゼロ

1. CANバスが起動しているか確認:

   ```bash
   ip link show can0
   # state UP を確認
   ```

2. CANトラフィックがあるか確認:

   ```bash
   candump can0
   # データが流れていることを確認
   ```

3. センサーIDが正しいか確認:
   - `candump` で実際のCAN IDを確認
   - CAN IDの末尾（例: 004の"4"）がセンサーIDと一致している必要がある

### 半分のタクセルがゼロ

- センサーモデルが間違っている可能性
- `XR1944` → `uSPa44` に変更してみる
- CAN IDパターンとモデルの対応を確認

### FUSEエラー（AppImage実行時）

Docker内ではFUSEが使えないため、AppImageを展開して使用する:

```bash
./xela_server.AppImage --appimage-extract
mv squashfs-root software/v1.7.7/bin/server
./software/v1.7.7/bin/server/AppRun -f software/v1.7.7/config/xServ.ini
```

---

## バージョン情報

| コンポーネント | バージョン |
|---------------|-----------|
| xela_server | 1.7.7 (build 161703) |
| xela_service | 1.7.6_137302 |
| ROS | One (Noetic互換) |
| Ubuntu | 22.04 |

---

## 参考リンク

- XELA Software Manual v1.7.7
- <https://github.com/xela-robotics/xela_server_ros>

---

*最終更新: 2026-01-14*
