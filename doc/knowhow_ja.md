# 浦島効果デバイス ビルドノウハウ

## 概要

このドキュメントは浦島効果デバイスのビルドプロセスに関するノウハウと一般的な問題解決方法をまとめています。

## ビルド環境

- **Arduino IDE**: 最新版を使用
- **ボード**: M5Stack AtomS3R
- **ボード設定**: m5stack:esp32:m5stack_atoms3r
- **Arduino CLI**: C:\Program Files\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe

## ビルドコマンド

```bash
& "C:\Program Files\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe" compile --fqbn m5stack:esp32:m5stack_atoms3r .
```

## Arduino CLIの使用方法

Arduino CLIを使用すると、コマンドラインからArduinoスケッチのコンパイルとアップロードが可能になります。このプロジェクトでは以下のパスにArduino CLIがインストールされています：

```
C:\Program Files\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe
```

### 基本的なコマンド

#### コンパイル

```powershell
& "C:\Program Files\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe" compile --fqbn m5stack:esp32:m5stack_atoms3r .
```

#### 詳細なビルド情報を取得

```powershell
& "C:\Program Files\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe" compile --fqbn m5stack:esp32:m5stack_atoms3r . -v
```

#### アップロード

```powershell
& "C:\Program Files\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe" upload -p [PORT] --fqbn m5stack:esp32:m5stack_atoms3r .
```

※ [PORT]は実際のシリアルポート（例：COM3）に置き換えてください。

## ハードウェア接続

### 使用ハードウェア
- M5Stack AtomS3R
- M5Stack AtomicBase GPS
- microSDカード（オプション、データロギング用）

### 重要なピン設定
- **GPS接続**
  - GPS_TX_PIN: 5（GPSからデータを受信するAtomS3R GPIO5）
  - GPS_RX_PIN: -1（未接続）

- **SDカード接続**
  - SD_MOSI_PIN: 6（MOSIピン G6）
  - SD_MISO_PIN: 8（MISOピン G8）
  - SD_SCK_PIN: 7（CLKピン G7）
  - SD_CS_PIN: 38（CSピン G38）

**注意**: GPIO5のGPSモジュールとの競合を避けるため、SDカードのCSピンにはGPIO38を使用することが重要です。SDカードとディスプレイは同じSPIバスを共有しているため、適切に設定しないと競合が発生する可能性があります。

### AtomicBase GPS接続設定

AtomicBase GPSモジュールとM5Stack AtomS3Rの接続には以下の設定を使用します：

```
GPS_TX_PIN = 5
GPS_RX_PIN = -1
```

**重要な注意点**:
- M5Stack AtomS3RとAtomic GPS Baseの組み合わせでは、GPIO5をGPS_TX_PINとして使用します
- SDカードのCS_PINには別のGPIO（38）を使用して、GPIO5との競合を避けてください
- GPS_RX_PINは-1に設定されています（単方向通信）

### GPS-IMUセンサーフュージョン

速度測定の精度を向上させるため、GPSとIMUのセンサーフュージョンを実装しています：

```cpp
// GPS-IMUフュージョン設定
const float GPS_IMU_FUSION_WEIGHT = 0.2;  // GPSデータの重み（0.0〜1.0）
const unsigned long GPS_VALID_TIMEOUT = 5000;  // GPSデータの有効期限（ミリ秒）
```

**フュージョンの仕組み**:
1. GPSとIMUの両方のデータが有効な場合、重み付け平均を使用
2. GPSデータが無効または古い場合、IMUデータのみを使用
3. GPS_VALID_TIMEOUT（5000ms）以内に新しいGPSデータが受信されない場合、GPSデータは無効と見なされる

**実装例**:
```cpp
// 速度更新関数の例
void updateSpeed() {
  // GPSが有効かつタイムアウトしていない場合
  if (gpsValid && (millis() - lastGpsUpdate < GPS_VALID_TIMEOUT)) {
    // GPSとIMUのフュージョン
    currentSpeed = gpsSpeed * GPS_IMU_FUSION_WEIGHT + imuSpeed * (1 - GPS_IMU_FUSION_WEIGHT);
  } else {
    // IMUのみを使用
    currentSpeed = imuSpeed;
  }
}
```

### 電源
- **USB電源**: USB Type-Cケーブルを使用してAtomS3Rに接続
- **バッテリー**: AtomicBase GPSには内蔵バッテリーがあり、充電後に使用可能

### トラブルシューティング

#### ディスプレイが更新されない
ボタンを押してもディスプレイが正しく更新されない場合：
- GPSとSDカードの両方に対して正しいピン設定を使用していることを確認
- SD_CS_PINが38に設定されていることを確認（GPIO5とのGPS競合を避けるため）
- ボタン処理が`M5.BtnA.wasPressed()`ではなく`M5.BtnA.wasClicked()`を使用していることを確認（信頼性向上のため）
- `M5.update()`がloop()の先頭で呼び出されていることを確認

#### SDカードが動作しない
SDカードが検出されないか、ディスプレイの問題を引き起こす場合：
- SDカードが正しくフォーマットされていることを確認（FAT32）
- SDカードのピンが正しく設定されていることを確認
- SPI初期化が`SPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN)`で行われていることを確認
- SPI初期化後に`SD.begin(SD_CS_PIN)`を使用していることを確認

#### GPSがデータを受信しない
- GPS_TX_PINが5に設定されていることを確認
- GPS受信状態の良い場所に移動
- シリアル接続設定を確認（9600ボーレート）

### 注意事項
- AtomS3RとAtomicBase GPSは精密な電子機器です。静電気や物理的な衝撃から保護してください
- 高温多湿の環境での使用は避けてください
- SDカードの挿入や取り出しは、デバイスの電源がオフの状態で行うことをお勧めします

## ソフトウェア要件

- Arduino IDE
- 必要なライブラリ:
  - M5Unified (v0.1.6以降)
  - TinyGPS++
  - SPI
  - SD
  - FS
  - Preferences

### インストール手順

1. このリポジトリをクローンまたはダウンロード
2. Arduino IDEで`Urashima_Effect_Device.ino`を開く
3. Arduino ライブラリマネージャーを通じて必要なライブラリをすべてインストール
4. M5Stack AtomS3Rボード（m5stack:esp32:m5stack_atoms3r）を選択
5. コンパイルしてデバイスにアップロード

### 基本的な使用方法

1. デバイスの電源を入れて起動画面を確認
2. GPS信号を取得するために屋外または窓の近くに移動
3. GPS信号が取得されると、現在の速度と時間膨張効果が表示される
4. 物理ボタンを短く押して表示モードを切り替え（メイン→GPS→IMU→メイン）
5. 物理ボタンを長押し（2秒）してリセット確認モードに入る

## 一般的なビルドエラーと解決方法

### 1. 関数宣言と定義の不一致

**エラー例**:
```bash
function expects X arguments, Y provided
```

**解決方法**:
- ヘッダファイル(.h)での関数宣言と、実装ファイル(.cpp)での関数定義の引数が一致しているか確認
- 呼び出し側のコードが正しい数の引数を渡しているか確認

### 2. 未定義のシンボル

**エラー例**:
```bash
'SYMBOL_NAME' was not declared in this scope
```

**解決方法**:
- 必要なヘッダファイルがインクルードされているか確認
- 定数名が正しいか確認（例：TFT_GRAYではなくTFT_DARKGREYを使用）
- 名前空間の指定が必要な場合は追加

### 3. 関数の重複定義

**エラー例**:
```bash
'function_name' previously defined here
```

**解決方法**:
- 同じ関数が複数の場所で定義されていないか確認
- クラスメソッドの場合、クラス名::メソッド名の形式で定義されているか確認

### 4. メンバー関数の未定義

**エラー例**:
```bash
'CLASS_NAME' has no member named 'FUNCTION_NAME'
```

**解決方法**:
- クラスのヘッダファイルに関数が宣言されているか確認
- 関数名のスペルミスがないか確認
- 継承関係を確認（親クラスのメソッドを呼び出している場合）

### 5. ライブラリの問題

**エラー例**:
```bash
No such file or directory
```

**解決方法**:
- 必要なライブラリがインストールされているか確認
- ライブラリのバージョンが互換性があるか確認
- インクルードパスが正しいか確認

### 6. C++のメソッド実装順序の問題

**エラー例**:
```bash
'methodName's scope
```

**解決方法**:
- C++では、メソッドの実装順序が重要です。あるメソッドが別のメソッドを呼び出す場合、呼び出されるメソッドを先に実装する必要があります。
- 循環参照がある場合は、前方宣言を使用するか、実装を分割する必要があります。
- クラスのメソッドをprivateからpublicに変更することで解決できる場合もあります。

### 7. M5Stackハードウェア固有の問題

**エラー例**:
```bash
'm5::M5Unified' has no member named 'dis'
```

**解決方法**:
- AtomS3Rなど特定のM5Stackデバイスでは、LEDやディスプレイの制御方法が異なります。
- AtomS3RではLEDの制御に`M5.dis`ではなく`M5.Lcd.fillScreen()`を使用します。
- ハードウェア固有の機能を使用する場合は、デバイスのドキュメントを確認してください。

### 8. 未実装メソッドの問題

**エラー例**:
```bash
undefined reference to 'ClassName::methodName'
```

**解決方法**:
- ヘッダファイル(.h)での関数宣言と、実装ファイル(.cpp)での関数定義の引数が一致していて、実装が行われているか確認
- 宣言と同じシグネチャ（引数の型と数、戻り値の型）で実装を追加します。
- 実装例:
  ```cpp
  void ClassName::methodName(type1 arg1, type2 arg2) {
    // 実装内容
  }
  ```

## デバッグのヒント

1. **シリアルモニタの活用**: 
   - 重要な変数の値や処理の流れをシリアル出力で確認
   - `Serial.print()` や `Serial.println()` を戦略的に配置

2. **段階的なビルド**:
   - 大きな変更を一度に行わず、小さな変更ごとにビルドして問題を特定

3. **コメントアウト**:
   - 問題のある可能性がある部分を一時的にコメントアウトして、エラーの原因を特定

4. **プリプロセッサの活用**:
   - `#ifdef DEBUG` などの条件付きコンパイルを使用してデバッグコードを管理

5. **段階的なエラー解決**:
   - エラーメッセージを注意深く読み、問題の箇所を特定します。
   - 複数のエラーがある場合は、最初のエラーから順に解決します。

6. **コードの簡略化**:
   - 複雑な実装を一時的に簡略化して、基本的な機能が動作するか確認します。
   - 動作確認後、段階的に元の実装に戻します。

7. **依存関係の確認**:
   - メソッド間の呼び出し関係を確認し、循環参照がないか確認します。
   - クラス間の依存関係を最小限に抑えます。

8. **ビルドフラグの活用**:
   - `-v`（詳細）フラグを使用して、より詳細なビルド情報を取得します。
   ```bash
   & "C:\Program Files\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe" compile --fqbn m5stack:esp32:m5stack_atoms3r . -v
   ```

## 特定の機能に関するノウハウ

### IMUセンサー

- AtomS3Rには、BMI270（6軸加速度・ジャイロセンサー）とBMM150（3軸地磁気センサー）の9軸センサーシステムが搭載
- 校正状態は3段階（GOOD、OK、POOR）で表示され、校正されていない場合は「NOT CALIBRATED」と表示

#### AtomS3R IMUセンサー詳細仕様
- **センサー特性**
  - BMI270
    - 加速度計: ±4G範囲、変換係数 8192.0f (INT16_to_G)
    - ジャイロスコープ: ±2000dps範囲、変換係数 16.384f (INT16_to_DPS)
  - BMM150
    - 磁力計: マイクロテスラ(uT)単位で測定

#### 校正機能実装ガイドライン
- **校正ステージ**
  - 3段階の校正プロセスを実装（磁力計、加速度計、ジャイロスコープ）
  - 校正状態表示: GOOD(3)、OK(2)、POOR(1)、NOT CALIBRATED(0)
  - 色分け表示: GOOD(緑)、OK(黄)、POOR(オレンジ)、NOT CALIBRATED(赤)

- **校正手順ガイダンス**
  - 磁力計: 8の字を描くように回転させる
  - 加速度計/ジャイロスコープ: デバイスをあらゆる方向にひっくり返す
  - 校正中はLEDを黄色に点滅させる視覚的フィードバック

- **校正の限界と注意点**
  - M5Unifiedライブラリの基本校正機能はオフセットバイアスのみを推定
  - 完全な校正にはスケールバイアスも測定が必要
  - 高度な校正にはMotionCalなどの外部ツールを推奨

#### IMUデータ表示ガイドライン
- **センサーデータ表示**
  - ジャイロスコープデータ: deg/s単位、緑色で表示
  - 加速度計データ: m/s²単位、黄色で表示
  - 磁力計データ: uT単位、マゼンタ色で表示

- **傾き表示の視覚化**
  - ピッチとロールを円の位置で視覚的に表現
  - 基準円（水平位置）と現在の傾き位置（緑の円）を表示
  - 最大±30度の傾きを円の半径に対応させる

- **データ構造設計**
  - IMUデータはフラットな構造体で管理
  - センサー種別ごとにX/Y/Z軸の値を明確に区別
  - デバッグ出力を整理し、シリアルコンソールで確認可能に

#### 参考実装リソース
- **公式リファレンス**
  - M5Stack AtomS3R工場デモコード: https://github.com/m5stack/M5AtomS3-UserDemo/tree/atoms3r
  - BMI270/BMM150実装例: https://github.com/m5stack/M5AtomS3-UserDemo/blob/atoms3r/platforms/atoms3r/main/hal_atom_s3r/utils/bmi270/src/bmi270.cpp

- **コミュニティリソース**
  - 9軸IMU実装例: https://github.com/bareboat-necessities/bbn-atomS3R/
  - 高度な磁力計校正ツール: https://github.com/bareboat-necessities/bbn-MotionCal

### GPS

#### GPS初期化と設定
- AtomicBase GPSモジュールはUART接続で使用
- 正しいピン設定が重要:
  - GPS_TX_PIN: 5（GPSからデータを受信するAtomS3R GPIO5）
  - GPS_RX_PIN: -1（未接続）
- シリアル通信設定: 9600bps, 8N1
- TinyGPS++ライブラリを使用してGPSデータを解析

#### GPS-IMUセンサーフュージョン
- GPS_IMU_FUSION_WEIGHT定数（0.2）を使用して、GPSとIMUデータを融合
- GPS_VALID_TIMEOUT（5000ms）を超えるとGPSデータが無効と判断し、IMUデータのみを使用
- 両方のデータが有効な場合は加重平均を使用して精度を向上

#### GPSデータ処理のベストプラクティス
- 常にGPSデータの有効性をチェック（`gps.location.isValid()`など）
- 無効なデータに対するフォールバックメカニズムを実装
- GPSデータの更新頻度を考慮したタイミング処理

### SDカードロギング

#### SDカード初期化
- SDカードとディスプレイの競合を避けるための正しいピン設定:
  - SD_MOSI_PIN: 6
  - SD_MISO_PIN: 8
  - SD_SCK_PIN: 7
  - SD_CS_PIN: 38（重要：GPIO5との競合を避けるため）
- 初期化方法:
  - SPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN)
  - SD.begin(SD_CS_PIN)

#### ログファイル管理
- ファイル名には日付や一意の識別子を含める
- ファイル開始時にヘッダー行を書き込む
- 新しいセッション開始時にセパレーター行を追加
- 定期的な書き込み（例：5秒ごと）でバッテリー消費とデータ詳細さのバランスを取る

#### エラー処理
- SDカード初期化失敗時の適切なエラー表示
- 書き込みエラー発生時のリカバリーメカニズム
- ファイルクローズの確実な実行

### 相対論的時間計算

#### 時間膨張計算
- ローレンツ因子を使用した計算: γ = 1/√(1-(v²/c²))
- 修正光速値（0.03 km/s）を使用して日常的な速度でも効果を体感可能に
- 速度が閾値（例：100 km/h）を超えた場合の特別な視覚的フィードバック

#### 時間表示
- デバイス時間（実時間）と相対論的時間の両方を表示
- 時間差を色分けして表示（正常: 黄色、異常: シアン、ゼロ: 緑）
- 時間リセット機能の実装（ボタン長押しによる確認画面表示）

#### 相対論的時間計算の仕組み

このデバイスはアインシュタインの特殊相対性理論に基づいて時間膨張を計算します。実際の光速（約300,000 km/s）では日常的な速度での時間膨張効果は知覚できないため、このデバイスは仮想の「修正光速」（0.03 km/sまたは108 km/h）を使用して効果を誇張しています。

#### 時間の用語説明
- **DEV（Device Time、デバイス時間）**: デバイスが測定する実際の経過時間。現実世界の時間と同じ速度で進む。
- **REL（Relativistic Time、相対論的時間）**: 特殊相対性理論に基づいて計算された時間。速度が上がるほど進み方が遅くなる。
- **DIFF（Difference、時間差）**: DEVとRELの差。時間膨張効果の大きさを示す。

時間膨張はローレンツ因子を使用して計算されます：

γ = 1/√(1-(v²/c²))

ここで：
- γ（ガンマ）は時間膨張因子
- vは現在の速度
- cは修正された光速（0.03 km/sまたは108 km/h）

速度が上がるにつれて、時間膨張効果はますます顕著になります：
- 30 km/hでは：時間は通常の約0.96倍の速度で経過（γ ≈ 1.04）
- 60 km/hでは：時間は通常の約0.87倍の速度で経過（γ ≈ 1.15）
- 90 km/hでは：時間は通常の約0.64倍の速度で経過（γ ≈ 1.56）
- 100 km/hでは：時間は通常の約0.48倍の速度で経過（γ ≈ 2.08）

### ユーザーインターフェース

#### ディスプレイモード
- メイン表示: 速度、時間情報、距離を表示
- GPS RAW表示: GPS生データ（日時、位置、衛星数など）を表示
- IMU RAW表示: 加速度、ジャイロ、推定速度データを表示

#### ボタン処理
- M5AtomS3Rは`BtnA`のみ使用可能
- `wasPressed()`より`wasClicked()`の方が信頼性が高い
- 長押し検出のための適切なタイマー実装

#### 視覚的フィードバック
- GPS信号強度の表示
- SDカード状態の表示
- 時間リセット確認画面の実装
- エラー状態の色分け表示

## パフォーマンスの最適化

1. **メモリ使用量の削減**:
   - 大きな配列や文字列は必要最小限に
   - 動的メモリ割り当ては避け、静的割り当てを優先

2. **描画処理の最適化**:
   - 必要な部分のみを更新
   - スプライトを使用したダブルバッファリングの活用

3. **電力消費の最適化**:
   - 不要なセンサーの電源を切る
   - 処理間隔を適切に設定

## 継続的な改善

- エラーが発生した場合は、このドキュメントに追加して知識を蓄積
- 解決方法を見つけたら、コードにコメントとして残すことも検討

## ビルドエラーの解決方法

### 1. 未実装メソッドによるビルドエラー

**エラー例**:
```bash
undefined reference to `ClassName::methodName(type1, type2, ...)'
```

**解決方法**:
- ヘッダファイル(.h)で宣言されているメソッドが実装ファイル(.cpp)で実装されていない場合に発生します。
- 宣言と同じシグネチャ（引数の型と数、戻り値の型）で実装を追加します。
- 実装例:
  ```cpp
  void ClassName::methodName(type1 arg1, type2 arg2) {
    // 実装内容
  }
  ```

### 2. メソッド名の不一致によるエラー

**エラー例**:
```bash
'methodName' was not declared in this scope
```

**解決方法**:
- クラス内で定義されているメソッド名と呼び出しているメソッド名が一致しているか確認します。
- 例: `methodA()`を呼び出している場合、実際のメソッド名が`methodB()`であれば、呼び出し側を修正します。
- M5Stackデバイスでは、ハードウェア固有のメソッド名に注意が必要です。

## コード構造説明

## 概要

浦島効果デバイスのコードは、Arduino環境で動作するC++で書かれた単一のスケッチファイル（`Urashima_Effect_Device.ino`）で構成されています。このセクションでは、コードの主要な部分と機能について説明します。

## 使用ライブラリ

```cpp
#include <M5Unified.h>    // M5Stackデバイス統一ライブラリ
#include <TinyGPS++.h>    // GPS解析ライブラリ
#include <SPI.h>          // SPIバス通信ライブラリ
#include <Preferences.h>  // 永続的ストレージライブラリ
#include <SD.h>           // SDカードライブラリ
```

## 定数定義

コードには以下の主要な定数が定義されています：

- **光速関連**:
  - `ORIGINAL_LIGHT_SPEED`: 実際の光速（299792.458 km/s）
  - `MODIFIED_LIGHT_SPEED`: 修正された光速（0.03 km/sまたは108 km/h）

- **GPS接続設定**:
  - `GPS_RX_PIN`: GPSからデータを受信するピン（5）
  - `GPS_TX_PIN`: GPS送信用ピン（-1、未接続）

- **SDカード設定**:
  - `SD_MOSI_PIN`: MOSIピン（6）
  - `SD_MISO_PIN`: MISOピン（8）
  - `SD_SCK_PIN`: SCKピン（7）
  - `SD_CS_PIN`: CSピン（38）
  - `SD_LOG_INTERVAL`: ログ記録間隔（5000ms）
  - `SD_LOG_FILENAME`: ログファイル名（"/urashima_log.csv"）

- **IMU関連設定**:
  - `GPS_IMU_FUSION_WEIGHT`: センサーフュージョンの重み（0.2）
  - `GPS_VALID_TIMEOUT`: GPS有効性タイムアウト（5000ms）
  - その他、加速度閾値、ジャイロ閾値、速度減衰係数など

## グローバル変数

主要なグローバル変数：

- **時間関連**:
  - `elapsed_device_time`: 経過デバイス時間（秒）
  - `elapsed_relativistic_time`: 経過相対論的時間（秒）
  - `time_difference`: 時間差（秒）
  - `time_dilation`: 時間膨張係数

- **速度・距離関連**:
  - `current_speed`: 現在の速度（km/h）
  - `total_distance`: 総移動距離（km）
  - `gpsImuFusedSpeed`: GPS-IMU融合速度

- **状態管理**:
  - `display_mode`: 表示モード（0:メイン, 1:GPS, 2:IMU）
  - `gpsDataReceived`: GPS信号受信状態
  - `sdCardInitialized`: SDカード初期化状態

## 主要関数

### 初期化と基本ループ

- **`setup()`**: デバイスの初期化を行います
  - M5デバイスの初期化
  - ディスプレイの初期化と初期画面表示
  - GPS通信の初期化
  - IMUの初期化
  - SDカードの初期化

- **`loop()`**: メインループ関数
  - ボタン状態の更新と処理
  - GPS・IMUデータの更新
  - 速度と移動距離の計算
  - 相対論的時間膨張の計算
  - ディスプレイの更新
  - SDカードへのデータログ記録（5秒ごと）
  - 永続的ストレージへのデータ保存（60秒ごと）

### 相対論的効果計算

- **`calculateRelativisticEffect()`**: 相対論的時間膨張効果を計算
  - 現在の速度に基づいてローレンツ因子を計算
  - デバイス時間と相対論的時間の更新
  - 時間差の計算

### 表示関連

- **`updateDisplay()`**: 現在の表示モードに応じてディスプレイを更新
  - メインモード: 速度、時間情報、距離を表示
  - GPSモード: GPS生データを表示
  - IMUモード: IMUセンサーデータを表示

- **`forceCompleteRedraw()`**: 画面全体を強制的に再描画

- **`formatTimeHMS()`**: 秒数を時:分:秒形式に変換するヘルパー関数

### IMU関連

- **`initializeImu()`**: IMUセンサーの初期化

- **`calibrateImu()`**: IMUセンサーのキャリブレーション
  - 加速度センサーとジャイロスコープのオフセット計算
  - キャリブレーション状態の表示

- **`updateImuData()`**: IMUデータの更新と処理
  - 加速度とジャイロデータの読み取り
  - 速度の推定計算
  - 静止状態の検出

### 速度と距離計算

- **`updateSpeed()`**: GPS・IMUデータに基づく速度の更新
  - GPS速度の読み取り
  - IMU速度の計算
  - GPS-IMUセンサーフュージョンの適用

- **`updateDistance()`**: 移動距離の計算と更新
  - 速度と経過時間に基づく距離の累積計算
  - 定期的な永続的ストレージへの保存

### データ永続化

- **`saveDataToPersistentStorage()`**: データを永続的ストレージに保存
  - 総移動距離
  - 最後の有効なGPS位置
  - IMUキャリブレーションデータ

- **`loadSavedData()`**: 永続的ストレージからデータを読み込み

### SDカードロギング

- **`initSDCard()`**: SDカードの初期化
  - SPIバスの設定
  - SDカードのマウント
  - ログファイルの準備

- **`logDataToSD()`**: データをSDカードにログ記録
  - 時間、速度、位置情報などをCSV形式で記録
  - 定期的なログ記録（5秒ごと）

## コードフロー

1. **起動時**:
   - ハードウェアの初期化（M5デバイス、ディスプレイ、GPS、IMU、SDカード）
   - 保存されたデータの読み込み
   - 初期画面の表示

2. **メインループ**:
   - ボタン状態の確認と処理
   - GPS・IMUデータの更新
   - 速度と移動距離の計算
   - 相対論的時間膨張の計算
   - ディスプレイの更新
   - SDカードへのデータログ記録（5秒ごと）
   - 永続的ストレージへのデータ保存（60秒ごと）

3. **ユーザー操作**:
   - ボタン短押し: 表示モードの切り替え（メイン→GPS→IMU→メイン）
   - ボタン長押し（2秒以上）: 時間リセット確認画面の表示
     - 確認画面でボタン押下: 時間と距離のリセット
     - 5秒間無操作: リセットキャンセル

## 拡張と修正のポイント

コードを拡張または修正する際の主要なポイント：

1. **定数定義**: 新しい機能やパラメータを追加する場合は、ファイル上部の定数定義セクションに追加

2. **表示関連**: `updateDisplay()`関数と`forceCompleteRedraw()`関数を修正して、新しい表示要素を追加

3. **データロギング**: `logDataToSD()`関数を修正して、新しいデータ項目をログに追加

4. **センサーフュージョン**: `updateSpeed()`関数でGPS-IMUセンサーフュージョンのロジックを調整

5. **時間計算**: `calculateRelativisticEffect()`関数で相対論的効果の計算を調整

6. **精度向上のヒント**:
   - millis()関数を使用した正確な時間計測
   - 浮動小数点計算の精度に注意（特に長時間の累積計算）
   - 定期的なリセットオプションの提供

## コードの簡略化

- 複雑な実装を一時的に簡略化して、基本的な機能が動作するか確認します。
- 動作確認後、段階的に元の実装に戻します。

## 依存関係の確認

- メソッド間の呼び出し関係を確認し、循環参照がないか確認します。
- クラス間の依存関係を最小限に抑えます。

## ビルドフラグの活用

- `-v`（詳細）フラグを使用して、より詳細なビルド情報を取得します。
  ```bash
  & "C:\Program Files\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe" compile --fqbn m5stack:esp32:m5stack_atoms3r . -v
  ```
