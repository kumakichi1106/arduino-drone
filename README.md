# Arduino DIY Drone - PID Control
 
システムエンジニアが電気・電子工作の知識ゼロから、ArduinoとMPU-6050を使ってドローンのPID姿勢制御を実装したプロジェクト。
 
宇宙・組み込み領域への転身を見据え、ハードウェア制御の基礎を学ぶことを目的として制作。
 
## デモ
 
[![Demo](https://img.youtube.com/vi/srNgrPQ5Ilg/0.jpg)](https://www.youtube.com/shorts/srNgrPQ5Ilg)
 
## 開発記録
 
制作過程はnoteに全5パート＋番外編で連載しています。
 
- [パート①](https://note.com/kumakichi0602/n/n4797e576f64c)
- [パート②](https://note.com/kumakichi0602/n/n61a693f59183)
- [パート2.5](https://note.com/kumakichi0602/n/na796f822a6bd)
- [パート③](https://note.com/kumakichi0602/n/ne98d9fb82eaf)
- [パート3.5](https://note.com/kumakichi0602/n/na29a9c0c967f)
- [パート④](https://note.com/kumakichi0602/n/n8268121d11f7)
- [パート⑤](https://note.com/kumakichi0602/n/n5397e2176758)
## 機能
 
- MPU-6050（6軸IMU）から現在の姿勢（Pitch / Roll / Yaw）を取得
- PID制御で4つのモーターの出力を自動調整
- シリアルモニタで姿勢とモーター出力をリアルタイム確認
## ハードウェア構成
 
| パーツ | 型番 | 備考 |
|---|---|---|
| マイコン | Arduino UNO R3（互換） | |
| ジャイロ・加速度センサー | MPU-6050 | I2C接続 |
| モーター | 8520 ブラシ付きDCモーター × 4 | 3〜5V, 0.15A |
| MOSFETトランジスタ | IRLZ44N × 4 |
| ダイオード | 1N4007 × 4 | 逆電流保護 |
| 電源モジュール | MB102 | 9V ACアダプター → 5V変換 |
| フレーム | AliExpress製 | |
 
## ソフトウェア構成
 
```
drone_pid.ino
├── setupMPU()        # MPU-6050の初期化（DMP使用）
├── checkAngles()     # センサーから現在の角度を取得
├── calculateErrors() # 目標角度と現在角度の誤差を計算
├── pidController()   # PID補正値を計算してモーター出力に反映
├── applyMotorSpeeds() # モーターに出力を適用
└── stopMotors()      # 全モーター停止
```
 
## PID制御について
 
目標角度（水平 = 0度）と現在角度の差（誤差）をもとに、4つのモーターの出力を自動調整して姿勢を安定させる制御方式。
 
```
誤差 = 目標角度 - 現在角度
 
P（比例）: 今どのくらいズレているか
I（積分）: ズレが積み重なっていないか
D（微分）: どのくらいの速さでズレているか
 
PID補正値 = Kp × 誤差 + Ki × 誤差の累積 + Kd × 誤差の変化量
```
 
各モーターへの補正値の割り当て：
 
```
左前(LF) = baseThrottle + pid[ROLL] + pid[PITCH] - pid[YAW]
右前(RF) = baseThrottle - pid[ROLL] + pid[PITCH] + pid[YAW]
左後(LB) = baseThrottle + pid[ROLL] - pid[PITCH] + pid[YAW]
右後(RB) = baseThrottle - pid[ROLL] - pid[PITCH] - pid[YAW]
```
 
## 使用ライブラリ
 
- [I2Cdev](https://github.com/jrowberg/i2cdevlib)
- [MPU6050_6Axis_MotionApps20](https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050)
## 開発環境
 
- Arduino IDE
- Arduino UNO R3（互換）
## Season1の結果と課題
 
PID制御の実装と動作確認には成功。しかしブラシ付きモーターが発生させる電気的ノイズがI2C通信（MPU-6050との通信）を妨害し、浮上には至らなかった。
 
**根本原因：** モーター端子間のセラミックコンデンサ（100nF）が未実装だったため、ブラシノイズを抑制できなかった。
 
## Season2の予定
 
- ブラシレスモーター + ESCへの変更
- カスタムPCBの設計・製造
- セラミックコンデンサによるノイズ対策
- 浮上
## License
 
MIT
