# M5StampFly LPWA Autonomous Flight Prototype

M5StampFlyを用いた、LPWAによる遠隔指令と自律移動を目的としたドローン制御プログラムです。

本プロジェクトは、M5Fly-kanazawaが公開している「M5StampFly Skeleton」をベースに、RCコントローラに依存しない自律飛行制御へ拡張しています。

## 概要

災害時や山間部など、Wi-Fiや携帯通信を利用しにくい環境において、LPWA通信を用いてドローンへ移動指令や目標位置を送信し、自律的に移動させることを目的としています。

姿勢制御には、角度制御と角速度制御を組み合わせたカスケード制御を使用しています。

```text
目標角度
   ↓
角度制御
   ↓
目標角速度
   ↓
角速度制御
   ↓
モータ出力
```

高度制御では、下向きToFセンサから取得した高度と高度変化量を使用して、推力指令を生成します。

## 主なファイル

### `src/main_loop.cpp`

周期制御処理を実行します。
姿勢制御、角速度制御、高度制御、モータ出力などの主要な飛行制御処理が記述されています。

### `src/sensor.cpp`

BMI270、BMM150、ToFセンサおよび電圧センサから値を取得し、姿勢角や高度などを計算します。

### `src/control.cpp`

角度制御、角速度制御、高度制御などの制御処理を実装します。

### `lib/BMM150_SensorAPI`

BMM150磁気センサを制御するためのライブラリです。

## 使用ハードウェア

| 機器        | 概要                  |
| --------- | ------------------- |
| M5StampS3 | ESP32-S3、8 MB Flash |
| BMI270    | 6軸IMU               |
| BMM150    | 3軸磁気センサ             |
| VL53L3CX  | 下向きToF距離センサ         |
| PMW3901   | オプティカルフローセンサ        |
| BMP280    | 気圧センサ               |
| INA3221   | 電流・電圧センサ            |
| LiHVバッテリー | 300 mAh             |

## 開発環境

* Visual Studio Code
* PlatformIO
* Arduino Framework
* M5StampFly
* ESP32-S3

## ベースプロジェクト

本プロジェクトは、以下のM5StampFly Skeletonをベースに開発しています。

* M5Fly-kanazawa / M5StampFly Skeleton
* M5Stack StampFly
* Kouhei Ito氏およびM5Stack社によるソースコード

元のライセンス表記および著作権表記は、各ソースファイルと`LICENSE`ファイルに従います。

## ライセンス

本プロジェクトに含まれる元プログラムはMIT Licenseで公開されています。

元プログラムの著作権表示およびライセンス表示は保持してください。
