#include "vex.h"

// ============ 底盘参数常量定义 ============
// EN: Drivetrain constants
// 轮子直径：2.5 英寸，电机减速比 1:2（电机转1圈，轮转2圈）
// EN: Wheel diameter 2.5 in; motor-to-wheel gearing 1:2 (one motor turn = two wheel turns)
const double W_DIA_IN = 2.5;
const double M2W_RATIO = 2.0;                     // 电机：轮 = 1:2
// EN: motor : wheel = 1:2
const double W_CIR = M_PI * W_DIA_IN * M2W_RATIO; // ≈ 15.71 英寸
// EN: about 15.71 inches
const double IN2M_DEG = 22.5; // 360.0 / W_CIR;  // ≈ 22.918 度/英寸
// EN: about 22.918 degrees per inch (the value actually used is a hand-tuned 22.5)
// =========================================

// ============ PIDController 实现 ============
// EN: PIDController implementation
// 计算控制量，传入目标值和当前测量值
// EN: Compute the control output from a target value and the current measurement
double PIDController::compute(double target, double current) {
  double err = target - current;
  // 积分防饱和：误差过大或方向变化时清零
  // EN: Integral anti-windup: reset the integral when the error is large or changes sign
  if (fabs(err) > 18 || err * lastError < 0) {
    integral = 0;
  } else {
    integral += err;
  }
  if (fabs(integral) > integralLimit) {
    integral = (integral > 0 ? integralLimit : -integralLimit);
  }
  derivative = err - lastError;
  lastError = err;
  lastintegral = integral;
  double out = err * kp + integral * ki + derivative * kd;
  if (fabs(out) > outputLimit) {
    out = (out > 0 ? outputLimit : -outputLimit);
  }
  return out;
}

// 在开始新运动前可以调用以清零历史状态
// EN: Call before starting a new motion to clear the history
void PIDController::reset() {
  lastError = 0;
  integral = 0;
  lastintegral = 0;
  derivative = 0;
}

// 全局用于行驶方向的 PID 控制器
// EN: Global PID controller used to hold heading while driving
PIDController headingPid;

// 转向 PID 参数（在 main() 中设置）
// EN: Turn PID gains (set in main())
// tkf：转向前馈，克服静摩擦；默认 0 表示不加前馈
// EN: tkf: turn feedforward to overcome static friction; 0 (default) means none
double tkp = 0, tki = 0, tkd = 0, tkf = 0;
double terr, tinteg, tderi, tlasterr, tpid;
int breakauto = 0; // 置 1 时中断正在运行的运动函数
// EN: Set to 1 to abort whatever motion function is running
bool c4m = false;

// 速度限幅到 -100~100
// EN: Clamp speed to -100..100
static double adjust_drive_speed(double speed) {
  if (speed > 100)
    return 100;
  if (speed < -100)
    return -100;
  return speed;
}

// 方便在主程序中初始化 PID 增益
// EN: Helper for setting the PID gains from main()
void set_heading_pid(double p, double i, double d) {
  headingPid.kp = p;
  headingPid.ki = i;
  headingPid.kd = d;
  headingPid.reset();
}

// 一级封装：沿指定角度移动，speed 为期望速度。
// EN: Level-1 wrapper: drive along the given heading at the desired speed.
// 使用之前先调用 set_heading_pid() 配置 PID 增益。
// EN: Call set_heading_pid() first to configure the PID gains.
void run_dir_speed(double degree, double speed) {
  wait(4, msec);
  // 获取当前航向并计算 PID 校正量
  // EN: Read the current heading and compute the PID correction
  double current = BrainInertial.rotation(degrees);
  double correction = headingPid.compute(degree, current);
  int left = adjust_drive_speed(speed + correction);
  int right = adjust_drive_speed(speed - correction);
  move(left, right);
}

// 沿指定角度走指定的距离（英寸）
// EN: Drive a given distance (inches) along a heading
// degree: 目标角度
// EN: degree: target heading
// speed: 电机速度（-100~100%）
// EN: speed: motor speed (-100..100%)
// dis: 行进距离（英寸）
// EN: dis: distance to travel (inches)
// st: true时到达后停止，false时保持最后速度
// EN: st: true = stop on arrival, false = keep the last speed
void run_dir_speed_dis(double degree, double speed, double dis, bool st) {
  leftM.setPosition(0, degrees);
  rightM.setPosition(0, degrees);
  // 将距离（英寸）转换为电机度数
  // EN: Convert distance (inches) to motor degrees
  double targetMotorDegrees = fabs(dis) * IN2M_DEG;
  double decelThreshold = targetMotorDegrees * 0.15; // 最后15%距离开始减速
  // EN: Start slowing down over the last 15% of the distance
  while (1) {
    double avgPos =
        (fabs(leftM.position(degrees)) + fabs(rightM.position(degrees))) / 2;
    double remaining = targetMotorDegrees - avgPos;
    if (avgPos >= targetMotorDegrees)
      break;
    // 动态减速：剩余距离越少，速度越小（线性减速在最后15%）
    // EN: Dynamic deceleration: the less distance left, the slower (linear ramp over the last 15%)
    double currentSpeed = speed;
    if (remaining < decelThreshold) {
      currentSpeed = speed * (remaining / decelThreshold); // 线性减速
      // EN: linear ramp-down
      if (fabs(currentSpeed) < 27) // 最少保持27速度
      // EN: never go below speed 27
      {
        currentSpeed = 27 * (currentSpeed > 0 ? 1 : -1);
      }
    }
    run_dir_speed(degree, currentSpeed);
    if (breakauto == 1)
      break;
  }
  if (st == true) {
    move(0, 0);
  }
}

void run_dir_speed_time(double degree, double speed, double time, bool st) {
  Brain.Timer.reset();
  while (Brain.Timer.value() < time) {
    run_dir_speed(degree, speed);
    if (breakauto == 1) {
      break;
    }
  }
  if (st == true) {
    move(0, 0);
  }
}

// 转向到指定角度
// EN: Turn to the given heading
// ff：前馈增益；传入负数（默认 -1）时使用全局 tkf
// EN: ff: feedforward gain; pass a negative number (default -1) to use the global tkf
void turnto(int degree, double ff) {
  double kf = (ff < 0) ? tkf : ff;
  int count = 0;
  tinteg = 0;
  tderi = 0;
  tlasterr = 0; // 重置状态变量
  // EN: reset state variables
  while (1) {
    if (breakauto == 1)
      break;
    wait(5, msec);
    terr = degree - BrainInertial.rotation(degrees);
    if (fabs(terr) > 17 ||
        terr * tlasterr < 0) // 误差过大或误差方向改变时清零积分
        // EN: reset the integral when the error is large or changes direction
    {
      tinteg = 0;
    } else {
      tinteg += terr;
    }
    // 积分限幅
    // EN: clamp the integral
    if (fabs(tinteg) > 7000) {
      tinteg = tinteg / fabs(tinteg) * 7000;
    }
    tderi = terr - tlasterr;
    tpid = terr * tkp + tinteg * tki + tderi * tkd;
    // 前馈：按误差方向加一个固定量，克服静摩擦
    // EN: Feedforward: add a fixed amount in the direction of the error to overcome static friction
    if (terr > 0)
      tpid += kf;
    else if (terr < 0)
      tpid -= kf;
    // 收敛判断：误差小于4度并保持20个周期
    // EN: Done when the error stays under 4 degrees for 20 cycles (about 100 ms)
    if (fabs(terr) < 4) {
      count++;
      if (count > 20) {
        break;
      }
    } else {
      count = 0; // 误差增大时重置计数
      // EN: reset the counter if the error grows again
    }
    if (tpid > 80) {
      tpid = 80;
    }
    if (tpid < -80) {
      tpid = -80;
    }
    tmove(tpid, -tpid);
    tlasterr = terr;
  }
  move(0, 0);
}

void move(int a, int c) {
  leftM.spin(forward, a, percent);
  rightM.spin(forward, c, percent);
}

void tmove(int a, int c) {
  leftM.spin(forward, a, rpm);
  rightM.spin(forward, c, rpm);
}

void delay(double t) {
  Brain.Timer.reset();
  while (Brain.Timer.value() < t) {
    wait(1, msec);
    if (breakauto == 1) {
      break;
    }
  }
}
