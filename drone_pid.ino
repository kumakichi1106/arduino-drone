#include <Wire.h>    // I2C通信ライブラリ
#include "I2Cdev.h"  // MPU6050用ライブラリ
#include "MPU6050_6Axis_MotionApps20.h"

// ピン番号
#define MOTOR_LF 11  // 左前
#define MOTOR_RF 9   // 右前
#define MOTOR_LB 10  // 左後
#define MOTOR_RB 6   // 右後

// 軸のインデックス
#define YAW 0
#define PITCH 1
#define ROLL 2

// 割り込みピン
#define INTERRUPT_PIN 2

// MPU6050関連
MPU6050 mpu;                         // MPU6050オブジェクト
bool dmpReady = false;               // 初期化成功フラグ
uint8_t mpuIntStatus;                // 割り込みステータス
uint8_t devStatus;                   // 初期化結果（0=成功）
uint16_t packetSize;                 // DMPパケットサイズ
uint16_t fifoCount;                  // FIFOバッファのデータ数
uint8_t fifoBuffer[64];              // FIFOバッファ
volatile bool mpuInterrupt = false;  // データ準備完了フラグ
Quaternion q;                        // クォータニオン（角度計算用）
VectorFloat gravity;                 // 重力ベクトル（角度計算用）

// 現在の角度 [Roll, Pitch, Yaw]センサーから読み取った値
float currentAngles[3] = { 0, 0, 0 };

// 目標角度(水平)
float target[3] = { 0, 0, 0 };

// PIDゲイン [Roll, Pitch, Yaw](0からにしようと思ったが、ネットに転がっているものを拝借)
float Kp[3] = { 0, 0.9, 0.9 };
;                                 // 比例
float Ki[3] = { 0.0, 0.0, 0.0 };  // 積分
float Kd[3] = { 0, 13, 13 };      // 微分

// 誤差（目標角度 - 現在角度）[Roll, Pitch, Yaw]
float error[3] = { 0.0, 0.0, 0.0 };

// 積分（誤差を毎ループ足し続けた合計値）[Roll, Pitch, Yaw]
float error_sum[3] = { 0.0, 0.0, 0.0 };

// 前回の誤差（微分の計算に使う）[Roll, Pitch, Yaw]
float prev_error[3] = { 0.0, 0.0, 0.0 };

// PID補正値（P+I+Dの合計）[Roll, Pitch, Yaw]
float pid[3] = { 0.0, 0.0, 0.0 };

// モーター出力
int baseThrottle = 10;  // 基本出力（出力高すぎるとモーターが動かない）
int throttle_lf = 0;    // 左前モーター出力
int throttle_rf = 0;    // 右前モーター出力
int throttle_lb = 0;    // 左後モーター出力
int throttle_rb = 0;    // 右後モーター出力

void dmpDataReady() {
  mpuInterrupt = true;
}


void setup() {
  stopMotors();
  Wire.begin();
  Serial.begin(115200);

  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  setupMPU();

  digitalWrite(13, LOW);
}

void loop() {
  if (!dmpReady) return;

  // センサーから角度を読む
  checkAngles();

  // 誤差を計算する
  calculateErrors();

  // PIDでモーター速度を計算して適用する
  pidController();
  applyMotorSpeeds();

  // デバッグ用
  Serial.print("Pitch: ");
  Serial.print(currentAngles[PITCH]);
  Serial.print(" Roll: ");
  Serial.print(currentAngles[ROLL]);
  Serial.print(" Yaw: ");
  Serial.print(currentAngles[YAW]);
  Serial.print(" LF: ");
  Serial.print(throttle_lf);
  Serial.print(" RF: ");
  Serial.print(throttle_rf);
  Serial.print(" LB: ");
  Serial.print(throttle_lb);
  Serial.print(" RB: ");
  Serial.println(throttle_rb);
}

// MPU6050の初期化(コピペです)
void setupMPU() {
  Wire.setClock(100000);  // I2C通信速度を100kHzに設定
  mpu.initialize();       // MPU6050を起動

  devStatus = mpu.dmpInitialize();  // DMP（デジタルモーションプロセッサ）を初期化

  // ジャイロのオフセット
  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);

  if (devStatus == 0) {
    // 初期化成功
    mpu.setDMPEnabled(true);
    // デジタルローパスフィルターを設定（ノイズ低減）
    mpu.setDLPFMode(MPU6050_DLPF_BW_20); // 20Hz以上のノイズをカット
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
    Serial.println("MPU OK!");
  } else {
    // 初期化失敗
    Serial.print("MPU Error: ");
    Serial.println(devStatus);
  }
}

// 現在の角度を読み取る
void checkAngles() {
  // センサーデータ読み取り（DMPを利用）
  if (mpu.getFIFOCount() < packetSize) return;

  mpu.getFIFOBytes(fifoBuffer, packetSize);
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(currentAngles, &q, &gravity);

  // ラジアンから度に変換
  currentAngles[YAW] = currentAngles[YAW] * 180 / M_PI;
  currentAngles[PITCH] = currentAngles[PITCH] * 180 / M_PI;
  currentAngles[ROLL] = currentAngles[ROLL] * 180 / M_PI;
}

// 誤差を計算する
void calculateErrors() {
  error[ROLL] = target[ROLL] - currentAngles[ROLL];
  error[PITCH] = target[PITCH] - currentAngles[PITCH];
  error[YAW] = target[YAW] - currentAngles[YAW];
}

// PID制御
void pidController() {
  float delta_err[3] = { 0, 0, 0 };  // 誤差の変化量

  // 積分：誤差を毎ループ足し続ける
  error_sum[ROLL] += error[ROLL];
  error_sum[PITCH] += error[PITCH];
  error_sum[YAW] += error[YAW];

  // 微分：今回の誤差と前回の誤差の差
  delta_err[ROLL] = error[ROLL] - prev_error[ROLL];
  delta_err[PITCH] = error[PITCH] - prev_error[PITCH];
  delta_err[YAW] = error[YAW] - prev_error[YAW];

  // 前回の誤差を更新
  prev_error[ROLL] = error[ROLL];
  prev_error[PITCH] = error[PITCH];
  prev_error[YAW] = error[YAW];

  // PID補正値を計算（P + I + D）
  pid[ROLL] = (error[ROLL] * Kp[ROLL]) + (error_sum[ROLL] * Ki[ROLL]) + (delta_err[ROLL] * Kd[ROLL]);
  pid[PITCH] = (error[PITCH] * Kp[PITCH]) + (error_sum[PITCH] * Ki[PITCH]) + (delta_err[PITCH] * Kd[PITCH]);
  pid[YAW] = (error[YAW] * Kp[YAW]) + (error_sum[YAW] * Ki[YAW]) + (delta_err[YAW] * Kd[YAW]);

  // 各モーターにPID補正値を適用
  // Roll：左右のモーターで差をつける
  // Pitch：前後のモーターで差をつける
  // Yaw：対角のモーターで差をつける
  throttle_lf = baseThrottle + pid[ROLL] + pid[PITCH] - pid[YAW];
  throttle_rf = baseThrottle - pid[ROLL] + pid[PITCH] + pid[YAW];
  throttle_lb = baseThrottle + pid[ROLL] - pid[PITCH] + pid[YAW];
  throttle_rb = baseThrottle - pid[ROLL] - pid[PITCH] - pid[YAW];

  // 0〜255の範囲に制限
  throttle_lf = limit(throttle_lf, 0, 100);
  throttle_rf = limit(throttle_rf, 0, 100);
  throttle_lb = limit(throttle_lb, 0, 100);
  throttle_rb = limit(throttle_rb, 0, 100);

}

// 値を最小値〜最大値の範囲に収める
float limit(float value, float min_value, float max_value) {
  if (value > max_value) return max_value;
  if (value < min_value) return min_value;
  return value;
}

// モーターに出力を適用する
void applyMotorSpeeds() {
  analogWrite(MOTOR_LF, throttle_lf);
  analogWrite(MOTOR_RF, throttle_rf);
  analogWrite(MOTOR_LB, throttle_lb);
  analogWrite(MOTOR_RB, throttle_rb);
}

// 全モーターを停止する
void stopMotors() {
  analogWrite(MOTOR_LF, 0);
  analogWrite(MOTOR_RF, 0);
  analogWrite(MOTOR_LB, 0);
  analogWrite(MOTOR_RB, 0);
}