# RM-PP03

STRV's Studio RM-PP03用ファームウェア

# 開発環境

- STM32CubeMX
  - 6.9.2 or Higher
- STM32Cube MCU Package for STM32F0 Series
  - 1.6.1

## 動作確認環境

- STM32CubeCLT
- VSCode
  - STM32 VS Code Extension

# ビルド手順

1. STM32CubeMX で ioc ファイルを開く
1. GENERATE CODE でコード生成する
1. 各自環境でコンパイル

# Modbus設定

## プロトコル詳細
- Modbus RTU
  - ボーレート 115200
  - パリティなし
  - スタートビット 1
  - ストップビット 1
  - データビット 8

※ASCIIには非対応

## コイル

アドレス | 割り当て | 内容
----- | ----- | -----
1 | 出力切り替え | 1:有効 / 0:無効
2 | 非常停止モード | 1:有効 / 0:無効

## 入力

アドレス | 割り当て | 内容
----- | ----- | -----
1 | 非常停止スイッチ状態 | 1:押下 / 0:無操作
2 | モードスイッチ状態 | 1 / 0

## 入力レジスタ

アドレス | 割り当て | 内容
----- | ----- | -----
1 | 平均電源電圧 | uint16電圧[mV]
2 | ピーク電源電圧 | uint16電圧[mV]
3 | 平均出力電流 | uint16電流[mA]
4 | ピーク出力電流 | uint16電流[mA]

## 保持レジスタ

アドレス | 割り当て | 内容
----- | ----- | -----
1 | 出力設定 | ※１
2 | 常点灯出力割合 | uint16
3 | 低周波重畳割合 | uint16

### ※１出力設定

ビット | 内容
----- | -----
0:14 | 割合 0 ～ 32767
15 | 方向 0 : 前進 / 1 : 後進

速度0のまま方向を入れ替えることで停止したまま灯火類の切り替えが可能

#### 例

- 0x0000 : 進行方向前で速度0
- 0x8000 : 進行方向後で速度0
- 0x0010 : 進行方向前で速度16
- 0xFFFF : 進行方向後ろで速度32767

# 使い方

## Modbus設定の流れ

1. 平均電源電圧を読み、電源が供給されていることを確認する
1. 非常停止を解除する
1. 出力を有効にする
1. 出力割合を上げて走行させる

## 常点灯の調整方法

1. 出力を有効にする
1. 出力割合を0にする
1. 常点灯を車両が走行するまで上げる
1. 常点灯を車両が停止するまで下げる

この調整を行うことで、車両が走り始めないギリギリの値で常点灯することができる。稀に走行抵抗の影響などにより、以降の走行時に出力割合を0にしても車両が止まらない場合がでるため、4の下げ幅は余裕を持つと良い。

# ハードウェア情報

## 接続方法

![RM-PP03s](doc/image/RM-PP03s_connections.png)

### 電源

電源入力コネクタに別売のACアダプタを接続する。

対応ACアダプタ
- 外径5.5mm
- 内径2.1mm
- センタープラス

推奨仕様
- 12V (Nゲージ) / 15,16V (HOゲージ)
- 1A以上 (フル室内灯付きなら2A等大きくする)

製品例
- 12V 1A
  - https://akizukidenshi.com/catalog/g/gM-17429/
- 12V 2A
  - https://akizukidenshi.com/catalog/g/gM-10659/
- 16V 3.75A
  - https://akizukidenshi.com/catalog/g/gM-10664/

### レール給電

端子台を利用してフィーダに接続する。

### PC通信

USB Cケーブルを利用して接続する。
バスパワー給電でマイコンは動作する。

# その他利用方法

## 非常停止

基板上の非常停止スイッチを押すと、出力を無効化し即時停止する。
外部接続コネクタには基板上のスイッチの端子が並列に引き出されているため、基板を箱に収めた場合などに外部に非常停止スイッチを設置するときに利用可能。

## 外部通信機能

将来拡張用

## ファームウェア更新方法

基板上の書き込みスイッチを押しながらUSBを接続することで、ファームウェア更新モードに入る。
github上のリリースページからダウンロードしたバイナリを [STM32CubeProgrammer](https://www.st.com/ja/development-tools/stm32cubeprog.html) により書き込む。
書き込みは手段はUARTを選択し、それ以外はデフォルト値でよい。


# 今後の予定

- 電流制限機能
- 外部通信機能