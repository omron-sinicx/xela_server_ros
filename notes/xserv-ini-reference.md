# xServ.ini 設定リファレンス

このドキュメントは `xServ.ini` の全設定パラメータを詳細に説明します。

---

## 設定ファイルの場所

```
xela_server_ros/config/xServ.ini
```

この場所は `.gitignore` で無視されているため、各環境で個別に設定が必要です。

---

## セクション一覧

| セクション | 説明 |
|-----------|------|
| `[server]` | サーバー/CAN設定（1.7.6以降、[CAN]を上書き）|
| `[CAN]` | CAN設定（レガシー）|
| `[viz]` | ビジュアライザ設定 |
| `[debug]` | デバッグ設定 |
| `[sensor]` | デフォルトセンサー設定 |
| `[sensorN]` | N番目のセンサーの個別設定（[sensor]を上書き）|

---

## [server] セクション

### bustype

CANバスのドライバータイプを指定。

| 値 | OS | 説明 |
|-----|-----|------|
| `socketcan` | Linux | 標準的なLinux CANドライバー（推奨）|
| `pcan` | Linux/Windows | PEAK CAN デバイス |
| `esd` | Windows | ESD CAN-USB/2, CAN-USB/3 |
| `slcan` | Windows | シリアルCAN |
| `sim` | 全て | シミュレーションモード（テスト用）|

```ini
bustype = socketcan
```

### channel

CANインターフェース名またはチャンネル。

| bustype | channel例 | 説明 |
|---------|----------|------|
| socketcan | `can0`, `slcan0` | インターフェース名 |
| pcan | `PCAN_USBBUS1` | PCANデバイス名 |
| esd | `0`, `1` | チャンネル番号 |
| sim | `sequential`, `parallel` | シミュレーションモード |

```ini
channel = can0
```

マルチCANの場合（カンマ区切り、スペースなし）:

```ini
channel = can0,can1
```

---

## [sensor] セクション

### num_of_sensor

接続されているセンサーの数。

```ini
num_of_sensor = 2
```

### model

センサーモデル名。CAN IDフォーマットを決定する重要な設定。

| モデル | サイズ | CAN IDフォーマット | 備考 |
|--------|--------|-------------------|------|
| `uSPa44` | 4×4 | 0X4, 1X4, 2X4, 3X4 | 新しいフォーマット（推奨）|
| `uSPa46` | 4×6 | - | 4×6センサー |
| `uSPa22` | 2×2 | - | 2×2センサー |
| `uSPa21` | 2×1 | - | 2×1センサー |
| `uSPa11` | 1×1 | - | 1×1センサー |
| `XR1944` | 4×4 | 0X4, 0X4+40, 1X4, 1X4+40 | 旧フォーマット |
| `XR1844` | 4×4 | - | 旧4チャンネル（販売終了）|

**注意**: センサーのラベルが「XR1944」でも、CAN IDフォーマットによっては `uSPa44` を設定する必要がある。

```ini
model = uSPa44
```

### id

コントローラーID（10進数で0〜15）。CAN IDの一部として使用される。

```ini
id = 4
```

**CAN IDの確認方法**:

```bash
candump can0
# 出力例: can0  004   [8]  93 7E 31 7E...
#              ^^^
#              この末尾の "4" がセンサーID
```

### channel（センサー用）

I2Cチャンネル（SDA）。複数の小型センサーを同一コントローラーで使用する場合に設定。

```ini
channel = 0
```

### rotation

センサーの回転。

| 値 | 説明 |
|----|------|
| `0` | 回転なし（デフォルト）|
| `1` | 90度時計回り |
| `2` | 180度 |
| `3` | 90度反時計回り |

```ini
rotation = 0
```

### version / ctr_ver

コントローラーバージョン。

| 値 | 説明 |
|----|------|
| `1` | 2018年〜2019年初期のみ |
| `2` | 15ビット解像度 |
| `3` | 16ビット解像度（デフォルト）|

```ini
version = 3
```

---

## モジュール設定

### calibration

力キャリブレーションモジュール。生データをニュートン単位の力に変換。

```ini
calibration = on
```

**出力**:

- WebSocket: `calibrated` フィールド（48値 = 16タクセル × 3軸）
- ROS: `forces` フィールド

### clustering

クラスタリングモジュール。複数タクセルへの接触を1点として計算。

```ini
clustering = on
```

**出力**:

- WebSocket: `extradots` フィールド `[[X, Y, 力], ...]`

### magcomp

磁気補正モジュール。**対応センサーのみ**。

```ini
magcomp = on
```

**警告**: 非対応センサーで有効化すると無効なデータが出力される。

### rawmode

磁気補正使用時に生データも出力。

```ini
magcomp = on
rawmode = on
```

---

## [sensorN] セクション

2番目以降のセンサーの個別設定。`[sensor]` の設定を上書きする。

```ini
[sensor]
num_of_sensor = 2
model = uSPa44
id = 4
calibration = on
clustering = on

[sensor2]
id = 6
# 他のパラメータは [sensor] を継承
```

複数センサーの例:

```ini
[sensor]
num_of_sensor = 3
model = uSPa44
id = 4

[sensor2]
id = 6

[sensor3]
id = 8
model = uSPa22  # 異なるモデルも可
```

---

## [viz] セクション

ビジュアライザ（xela_viz）用の設定。

| パラメータ | デフォルト | 説明 |
|-----------|-----------|------|
| `max_offset` | 100 | オフセット最大値 |
| `max_size` | 100 | サイズ最大値 |
| `arrows` | `off` | 矢印表示（`half`, `full`, `off`）|
| `grid` | `on` | グリッド表示 |
| `transparency` | `off` | 透明度 |
| `origins` | `off` | 原点表示 |
| `fps` | `off` | フレームレート表示 |
| `ups` | `off` | タクセル更新レート表示 |
| `temp_show` | `on` | 温度表示（対応センサーのみ）|

---

## [debug] セクション

デバッグ出力の設定。

### sens_print

センサー情報の出力レベル。

| 値 | 説明 |
|----|------|
| `full` | 全情報を出力 |
| `minimal` | 最小限の情報 |
| `off` | 出力しない |

```ini
[debug]
sens_print = full
```

---

## 完全な設定例

### 2センサー + キャリブレーション + クラスタリング

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

### シミュレーションモード

```ini
[server]
bustype = sim
channel = sequential

[sensor]
num_of_sensor = 1
model = uSPa44
id = 0
```

---

## 設定変更時の注意

1. **サーバーの再起動が必要**: 設定変更後は `xela_server` を再起動する
2. **ポートの解放**: 再起動前に前のプロセスが終了していることを確認

   ```bash
   pkill -9 -f xela_server
   ss -tlnp | grep 5000  # 何も表示されなければOK
   ```

3. **CAN IDの確認**: センサーモデルを変更した場合、`candump can0` で実際のCAN IDと一致しているか確認

---

*最終更新: 2026-01-13*
