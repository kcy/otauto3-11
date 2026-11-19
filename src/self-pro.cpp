#include "vex.h"

// ============ 底盘参数常量定义 ============
// 轮子直径：2.5 英寸，电机减速比 1:2（电机转1圈，轮转2圈）
const double W_DIA_IN = 2.5;
const double M2W_RATIO = 2.0;                     // 电机：轮 = 1:2
const double W_CIR = M_PI * W_DIA_IN * M2W_RATIO; // ≈ 15.71 英寸
const double IN2M_DEG = 22.5; // 360.0 / W_CIR;  // ≈ 22.918 度/英寸
// =========================================

// ============ PIDController 实现 ============
// 计算控制量，传入目标值和当前测量值
double PIDController::compute(double target, double current) {
  double err = target - current;
  // 积分防饱和：误差过大或方向变化时清零
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
void PIDController::reset() {
  lastError = 0;
  integral = 0;
  lastintegral = 0;
  derivative = 0;
}

// 全局用于行驶方向的 PID 控制器
PIDController headingPid;

// 转向 PID 参数（在 main() 中设置）
// tkf：转向前馈，克服静摩擦；默认 0 表示不加前馈
double tkp = 0, tki = 0, tkd = 0, tkf = 0;
double terr, tinteg, tderi, tlasterr, tpid;
int breakauto = 0; // 置 1 时中断正在运行的运动函数
bool c4m = false;

// 速度限幅到 -100~100
static double adjust_drive_speed(double speed) {
  if (speed > 100)
    return 100;
  if (speed < -100)
    return -100;
  return speed;
}

// 方便在主程序中初始化 PID 增益
void set_heading_pid(double p, double i, double d) {
  headingPid.kp = p;
  headingPid.ki = i;
  headingPid.kd = d;
  headingPid.reset();
}

// 一级封装：沿指定角度移动，speed 为期望速度。
// 使用之前先调用 set_heading_pid() 配置 PID 增益。
void run_dir_speed(double degree, double speed) {
  wait(4, msec);
  // 获取当前航向并计算 PID 校正量
  double current = BrainInertial.rotation(degrees);
  double correction = headingPid.compute(degree, current);
  int left = adjust_drive_speed(speed + correction);
  int right = adjust_drive_speed(speed - correction);
  move(left, right);
}

// 沿指定角度走指定的距离（英寸）
// degree: 目标角度
// speed: 电机速度（-100~100%）
// dis: 行进距离（英寸）
// st: true时到达后停止，false时保持最后速度
void run_dir_speed_dis(double degree, double speed, double dis, bool st) {
  leftM.setPosition(0, degrees);
  rightM.setPosition(0, degrees);
  // 将距离（英寸）转换为电机度数
  double targetMotorDegrees = fabs(dis) * IN2M_DEG;
  double decelThreshold = targetMotorDegrees * 0.15; // 最后15%距离开始减速
  while (1) {
    double avgPos =
        (fabs(leftM.position(degrees)) + fabs(rightM.position(degrees))) / 2;
    double remaining = targetMotorDegrees - avgPos;
    if (avgPos >= targetMotorDegrees)
      break;
    // 动态减速：剩余距离越少，速度越小（线性减速在最后15%）
    double currentSpeed = speed;
    if (remaining < decelThreshold) {
      currentSpeed = speed * (remaining / decelThreshold); // 线性减速
      if (fabs(currentSpeed) < 27) // 最少保持27速度
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
// ff：前馈增益；传入负数（默认 -1）时使用全局 tkf
void turnto(int degree, double ff) {
  double kf = (ff < 0) ? tkf : ff;
  int count = 0;
  tinteg = 0;
  tderi = 0;
  tlasterr = 0; // 重置状态变量
  while (1) {
    if (breakauto == 1)
      break;
    wait(5, msec);
    terr = degree - BrainInertial.rotation(degrees);
    if (fabs(terr) > 17 ||
        terr * tlasterr < 0) // 误差过大或误差方向改变时清零积分
    {
      tinteg = 0;
    } else {
      tinteg += terr;
    }
    // 积分限幅
    if (fabs(tinteg) > 7000) {
      tinteg = tinteg / fabs(tinteg) * 7000;
    }
    tderi = terr - tlasterr;
    tpid = terr * tkp + tinteg * tki + tderi * tkd;
    // 前馈：按误差方向加一个固定量，克服静摩擦
    if (terr > 0)
      tpid += kf;
    else if (terr < 0)
      tpid -= kf;
    // 收敛判断：误差小于4度并保持20个周期
    if (fabs(terr) < 4) {
      count++;
      if (count > 20) {
        break;
      }
    } else {
      count = 0; // 误差增大时重置计数
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
