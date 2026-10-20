#include <PinChangeInt.h>
#include <MsTimer2.h>
#include <BalanceCar.h>
#include <KalmanFilter.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include <util/atomic.h>
#include "gyro_correction.h"
#include "pid_controller.h"

MPU6050 mpu;
BalanceCar balancecar;
KalmanFilter kalmanfilter;

// 上次成功静止校准的原始 gx 均值。现在固定使用，不再等待自动校准。
const float gyroBiasX = -247.67f;
const float gyroBiasZ = 0.0f; // Z 轴尚未静止校准；需要时填入静止 gz 原始均值。
float angle0 = 0.00; // 机械平衡角补偿；目标俯仰角 = -angle0（度）。

// 保留当前增益数值；三个环的 Kp/Ki/Kd 现在全部参与计算，零值表示关闭该项。
// I 按误差*秒积分，D 按误差/秒计算；旧课程按采样累计的 Ki 不能直接照搬。
// 直立误差：angle+angle0（度）；D 使用校正后的 gyroX（度/秒）。
double kp = 20.0, ki = 0, kd = 1.0;
// 速度误差：setp0 - 滤波后的两轮总计数（每 40 ms 的计数，保持原有单位）。
double kp_speed = 3.0, ki_speed = 0.1, kd_speed = 0;
// 转向改为角速度 PID：误差=目标角速度-实测角速度；D 作用于角加速度。
// 与课程原 turnspin 中 kd_turn 直接乘角速度的含义不同。
double kp_turn = 0, ki_turn = 0, kd_turn = 0;
double setp0 = 0, dpwm = 0, dl = 0;
const float yawRateTarget = 0.0f; // 度/秒；零表示抑制自转，不是保持绝对航向。
float K1 = 0.05;
float Q_angle = 0.001, Q_gyro = 0.005;
float R_angle = 0.5, C_0 = 1;
float timeChange = 5;
float dt = timeChange * 0.001;

#define IN1M 7
#define IN2M 6
#define IN3M 13
#define IN4M 12
#define PWMA 9
#define PWMB 10
#define STBY 8
#define PinA_left 2
#define PinA_right 4

// 无串口/蓝牙运动命令；传给课程库的运动目标始终为零。
const int front = 0, back = 0, turnl = 0, turnr = 0, spinl = 0, spinr = 0;
int16_t ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
volatile long count_left = 0, count_right = 0;
int lz = 0, rz = 0, lpluse = 0, rpluse = 0;
int speedcc = 0, turncount = 0;
double Outputs = 0;
float turnoutput = 0;
PidController anglePid, speedPid, yawPid;
float speedFiltered = 0, yawRate = 0;

volatile bool imuOk = false;
bool filterSeeded = false;
bool imuAttempted = false;
unsigned long lastImuAttemptUs = 0;
volatile unsigned long imuErrors = 0;
volatile unsigned long debugCycles = 0, debugPeriodUs = 0, debugDurationUs = 0;
unsigned long debugPreviousStartUs = 0;
volatile byte debugStopFlags = 4; // bit0=角度保护，bit1=提起保护，bit2=IMU读数无效

struct DebugSnapshot
{
  unsigned long ms, cycles, periodUs, durationUs, errors;
  float angle, side, gyroX, speed, yaw;
  float angleIntegral, speedIntegral, yawIntegral;
  double pwmLeft, pwmRight, balanceOutput, speedOutput, rotation;
  int encLeft, encRight;
  int16_t accX, accY, accZ, rawGx, rawGy, rawGz;
  byte stopFlags, stby;
  bool imuValid;
};

void printRobotDebug()
{
  static unsigned long lastReportMs = 0;
  static DebugSnapshot sample;
  static byte part = 10;
  // 9600 baud 下每条约 330 字节。分段发送，避免占满 TX 后阻塞。
  // 只在 loop 中打印，5 ms 控制中断不进行串口输出。
  if (Serial.availableForWrite() < 60) return;
  if (part == 10)
  {
    const unsigned long now = millis();
    if (now - lastReportMs < 500) return;
    lastReportMs = now;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
      sample.ms = now;
      sample.cycles = debugCycles;
      sample.periodUs = debugPeriodUs;
      sample.durationUs = debugDurationUs;
      sample.errors = imuErrors;
      sample.angle = kalmanfilter.angle;
      sample.side = kalmanfilter.angle6;
      sample.gyroX = kalmanfilter.Gyro_x;
      sample.speed = speedFiltered;
      sample.yaw = yawRate;
      sample.angleIntegral = anglePid.iTerm;
      sample.speedIntegral = speedPid.iTerm;
      sample.yawIntegral = yawPid.iTerm;
      sample.pwmLeft = balancecar.pwm1;
      sample.pwmRight = balancecar.pwm2;
      sample.balanceOutput = balancecar.angleoutput;
      sample.speedOutput = Outputs;
      sample.rotation = turnoutput;
      sample.encLeft = lz;
      sample.encRight = rz;
      sample.accX = ax;
      sample.accY = ay;
      sample.accZ = az;
      sample.rawGx = gx;
      sample.rawGy = gy;
      sample.rawGz = gz;
      sample.stopFlags = debugStopFlags;
      sample.stby = digitalRead(STBY);
      sample.imuValid = imuOk;
    }
    part = 0;
  }

  switch (part)
  {
    case 0:
      Serial.print(F("DBG ms=")); Serial.print(sample.ms);
      Serial.print(F(" n=")); Serial.print(sample.cycles);
      break;
    case 1:
      Serial.print(F(" dt=")); Serial.print(sample.periodUs);
      Serial.print(F(" us=")); Serial.print(sample.durationUs);
      Serial.print(F(" IMU=")); Serial.print(sample.imuValid ? 1 : 0);
      Serial.print(F(" errs=")); Serial.print(sample.errors);
      break;
    case 2:
      Serial.print(F(" angle=")); Serial.print(sample.angle, 1);
      Serial.print(F(" raw="));
      Serial.print(atan2f(sample.accY, sample.accZ) * 57.3f, 1);
      Serial.print(F(" side=")); Serial.print(sample.side, 1);
      break;
    case 3:
      Serial.print(F(" gyroX=")); Serial.print(sample.gyroX, 2);
      Serial.print(F(" yawRate=")); Serial.print(sample.yaw, 2);
      break;
    case 4:
      Serial.print(F(" PWM=")); Serial.print((int)sample.pwmLeft);
      Serial.print(','); Serial.print((int)sample.pwmRight);
      Serial.print(F(" Bpid=")); Serial.print((long)sample.balanceOutput);
      Serial.print(F(" Vpid=")); Serial.print((long)sample.speedOutput);
      break;
    case 5:
      Serial.print(F(" Ypid=")); Serial.print((long)sample.rotation);
      Serial.print(F(" stop=")); Serial.print(sample.stopFlags);
      Serial.print(F(" STBY=")); Serial.print(sample.stby);
      Serial.print(F(" enc=")); Serial.print(sample.encLeft);
      Serial.print(','); Serial.print(sample.encRight);
      break;
    case 6:
      Serial.print(F(" acc=")); Serial.print(sample.accX);
      Serial.print(','); Serial.print(sample.accY);
      Serial.print(','); Serial.print(sample.accZ);
      Serial.print(F(" graw=")); Serial.print(sample.rawGx);
      Serial.print(','); Serial.print(sample.rawGy);
      Serial.print(','); Serial.print(sample.rawGz);
      break;
    case 7:
      Serial.print(F(" biasX=")); Serial.print(gyroBiasX, 2);
      Serial.print(F(" target=")); Serial.print(-angle0, 2);
      break;
    case 8:
      Serial.print(F(" I=")); Serial.print(sample.angleIntegral, 2);
      Serial.print(','); Serial.print(sample.speedIntegral, 2);
      Serial.print(','); Serial.print(sample.yawIntegral, 2);
      break;
    case 9:
      Serial.print(F(" v=")); Serial.print(sample.speed, 1);
      Serial.print(F(" vref=")); Serial.print(setp0, 1);
      Serial.print(F(" wref=")); Serial.println(yawRateTarget, 1);
      break;
  }
  ++part;
}

void countpluse()
{
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    lz = count_left;
    rz = count_right;
    count_left = 0;
    count_right = 0;
  }
  lpluse = lz;
  rpluse = rz;
  // 保留课程原有按逻辑 PWM 符号判断方向的方法。
  if (balancecar.pwm1 < 0 && balancecar.pwm2 < 0)
  {
    lpluse = -lpluse;
    rpluse = -rpluse;
  }
  else if (balancecar.pwm1 < 0 && balancecar.pwm2 > 0)
    lpluse = -lpluse;
  else if (balancecar.pwm1 > 0 && balancecar.pwm2 < 0)
    rpluse = -rpluse;
  balancecar.stopl += lpluse;
  balancecar.stopr += rpluse;
  balancecar.pulseleft += lpluse;
  balancecar.pulseright += rpluse;
}

bool readMotionSample()
{
  uint8_t bytes[14];
  Wire.clearWireTimeoutFlag();
  // 使用课程 I2Cdev 检查完整读取；失败时不把旧值当作新样本。
  if (I2Cdev::readBytes(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_XOUT_H,
                       14, bytes, 20) != 14 || Wire.getWireTimeoutFlag())
    return false;
  int16_t sample[6];
  for (byte i = 0; i < 6; ++i)
  {
    const byte offset = i < 3 ? 2 * i : 2 * i + 2; // 跳过温度寄存器。
    sample[i] = (int16_t)(((uint16_t)bytes[offset] << 8) | bytes[offset + 1]);
  }
  if (sample[0] == 0 && sample[1] == 0 && sample[2] == 0) return false;
  ax = sample[0]; ay = sample[1]; az = sample[2];
  gx = sample[3]; gy = sample[4]; gz = sample[5];
  return true;
}

void finishControlCycle(unsigned long started)
{
  debugDurationUs = micros() - started;
  ++debugCycles;
}

void resetPidControllers()
{
  anglePid.reset();
  speedPid.reset();
  yawPid.reset();
  balancecar.angleoutput = Outputs = turnoutput = 0;
  speedFiltered = 0;
  balancecar.pulseleft = balancecar.pulseright = 0;
  speedcc = turncount = 0;
  // 不清 stopl/stopr，避免破坏课程提起保护的保持/解除条件。
}

void updatePidControllers()
{
  // 保留原直立控制符号：正 angle 产生正 Bpid，混合器再取负。
  anglePid.update(kalmanfilter.angle + angle0, dt, kp, ki, kd,
                  true, kalmanfilter.Gyro_x);

  const bool speedUpdated = ++speedcc >= 8;
  if (speedUpdated) // 8 x 5 ms = 40 ms；不是角度环的 5 ms。
  {
    const float pulses = (float)balancecar.pulseleft + balancecar.pulseright;
    balancecar.pulseleft = balancecar.pulseright = 0;
    speedFiltered = 0.7f * speedFiltered + 0.3f * pulses;
    speedPid.update(setp0 - speedFiltered, dt * 8.0f,
                    kp_speed, ki_speed, kd_speed);
    speedcc = 0;
  }
  const bool yawUpdated = ++turncount >= 3;
  if (yawUpdated) // 3 x 5 ms = 15 ms。
  {
    yawPid.update(yawRateTarget - yawRate, dt * 3.0f,
                  kp_turn, ki_turn, kd_turn);
    turncount = 0;
  }

  // 对实际左右轮混合值抗积分饱和；只撤销加深饱和的本次 I 增量。
  // 直立/速度给两轮相同增量，转向给两轮相反增量；允许反向解饱和。
  const float left = -anglePid.output - speedPid.output - yawPid.output;
  const float right = -anglePid.output - speedPid.output + yawPid.output;
  const bool undoAngle = integralWorsensWheelSaturation(left, right,
                                                       anglePid.integralDelta(), false);
  const bool undoSpeed = speedUpdated && integralWorsensWheelSaturation(left, right,
                                                       speedPid.integralDelta(), false);
  const bool undoYaw = yawUpdated && integralWorsensWheelSaturation(left, right,
                                                       yawPid.integralDelta(), true);
  if (undoAngle) anglePid.undoIntegral();
  if (undoSpeed) speedPid.undoIntegral();
  if (undoYaw) yawPid.undoIntegral();
  balancecar.angleoutput = anglePid.output;
  Outputs = speedPid.output;
  turnoutput = yawPid.output;
}

void inter()
{
  const unsigned long started = micros();
  debugPeriodUs = debugPreviousStartUs == 0 ? 0 : started - debugPreviousStartUs;
  debugPreviousStartUs = started;
  sei(); // Wire 传输及编码器需要中断；MsTimer2 自身防止控制回调重入。
  countpluse();

  // 正常时每 5 ms 读取。故障时每 50 ms 重试，为串口监控留出执行时间。
  if (!imuAttempted || imuOk || started - lastImuAttemptUs >= 50000UL)
  {
    imuAttempted = true;
    lastImuAttemptUs = started;
    imuOk = readMotionSample();
    if (!imuOk) ++imuErrors;
  }
  if (!imuOk)
  {
    analogWrite(PWMA, 0);
    analogWrite(PWMB, 0);
    balancecar = BalanceCar();
    resetPidControllers(); // 恢复时不继承故障前积分/微分状态。
    filterSeeded = false;
    debugStopFlags = 4;
    finishControlCycle(started);
    return; // STBY 仍为 HIGH，日志持续输出；无有效传感器样本时 PWM 为零。
  }
  if (!filterSeeded)
  {
    kalmanfilter = KalmanFilter();
    kalmanfilter.angle = atan2f(ay, az) * 57.3f;
    kalmanfilter.angle6 = atan2f(ax, az) * 180.0f / PI;
    filterSeeded = true; // 从实际姿态开始，不把摆放姿态人为设成零度。
  }
  kalmanfilter.Angletest(ax, ay, az, gyroXForCourse(gx, gyroBiasX), gy, gz,
                        dt, Q_angle, Q_gyro, R_angle, C_0, K1);
  // 主程序用浮点缩放，保留课程 Z 轴正负号，避免库中的整数除法丢失小角速度。
  yawRate = -((float)gz - gyroBiasZ) / 131.0f;
  ++balancecar.posture;
  // 记录课程库原有保护条件，不改变其优先级或阈值。
  debugStopFlags = 0;
  if (kalmanfilter.angle > 30 || kalmanfilter.angle < -30) debugStopFlags |= 1;
  if ((kalmanfilter.angle6 > 3 ||
       (kalmanfilter.angle6 < -3 && turnl == 0 && turnr == 0 &&
        spinl == 0 && spinr == 0 && front == 0 && back == 0)) &&
      (balancecar.stopl + balancecar.stopr > 1500 ||
       balancecar.stopl + balancecar.stopr < -1500))
    debugStopFlags |= 2;
  if (debugStopFlags != 0)
    resetPidControllers();
  else
    updatePidControllers();
  // 两路方向引脚均相对上一版交换，反转电机极性；逻辑 PWM 保持不变。
  balancecar.pwma(Outputs, turnoutput, kalmanfilter.angle, kalmanfilter.angle6,
                  turnl, turnr, spinl, spinr, front, back, kalmanfilter.accelz,
                  IN1M, IN2M, IN3M, IN4M, PWMA, PWMB);
  finishControlCycle(started);
}

void Code_left() { ++count_left; }
void Code_right() { ++count_right; }

void setup()
{
  const byte pins[] = {IN1M, IN2M, IN3M, IN4M, PWMA, PWMB, STBY};
  for (byte i = 0; i < sizeof(pins); ++i) pinMode(pins[i], OUTPUT);
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
  digitalWrite(IN1M, LOW);
  digitalWrite(IN2M, LOW);
  digitalWrite(IN3M, LOW);
  digitalWrite(IN4M, LOW);
  digitalWrite(STBY, HIGH); // 驱动常使能；PWM 由平衡控制及原有保护决定。
  pinMode(PinA_left, INPUT);
  pinMode(PinA_right, INPUT);

  Serial.begin(9600);
  Wire.begin();
  Wire.setWireTimeout(10000, true);
  mpu.initialize();
  Serial.println(F("ROBOT_SERIAL_DEBUG v5.0-three-pid baud=9600"));
  Serial.print(F("IMU_CONNECTION="));
  Serial.println(mpu.testConnection() ? F("OK") : F("FAIL"));
  Serial.println(F("AUTO balance; STBY=1; no serial commands; DBG every 500ms."));
  Serial.println(F("gyroX=corrected deg/s; graw=raw x,y,z; target=-angle0."));
  Serial.println(F("PID: B=angle V=speed Y=yaw-rate; I=B,V,Y contributions."));
  Serial.println(F("v/vref=counts/40ms; yawRate/wref=deg/s; PID I/D use seconds."));
  Serial.println(F("stop bits: 1=angle 2=pickup 4=IMU; invalid IMU shows last sample."));
  Serial.flush();

  attachInterrupt(0, Code_left, CHANGE);
  attachPinChangeInterrupt(PinA_right, Code_right, CHANGE);
  MsTimer2::set(5, inter);
  MsTimer2::start();
}

void loop()
{
  printRobotDebug(); // 串口只输出；任何输入都不执行命令。
}
