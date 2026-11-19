struct PIDController {
  double kp{0};
  double ki{0};
  double kd{0};
  double lastError{0};
  double derivative{0};
  double integral{0};
  double lastintegral{0};
  double outputLimit{50};
  double integralLimit{2000};

  double compute(double target, double current);
  void reset();
};

extern PIDController headingPid;
extern double tkp;
extern double tki;
extern double tkd;
extern double tkf;
void set_heading_pid(double p, double i, double d);
void run_dir_speed(
    double degree,
    double
        speed); // 沿着x方向以x速度走，一级封装，不自带循环，需二级封装使用或者外面加条件循环使用
        // EN: Drive along heading `degree` at `speed`. Level-1 wrapper: no loop of its own; call it from a level-2 wrapper or from your own loop with an exit condition
void run_dir_speed_dis(double degree, double speed, double dis, bool st = true);
// 沿指定方向走指定距离。dis 单位为英寸；若沿用旧代码，使用 dis *
// OLD_DIS_TO_INCH 进行转换
// EN: Drive a set distance along a heading. `dis` is in inches; for old code use dis * OLD_DIS_TO_INCH (note: that constant does not exist in this project)
void run_dir_speed_time(double degree, double speed, double time,
                        bool st = true);
// 沿着x方向以x速度走多长时间，二级封装，带时间参数，st为true时结束后会停下来，false时结束后继续保持当前速度
// EN: Drive along a heading at a speed for `time` seconds. Level-2 wrapper; if st is true it stops at the end, if false it keeps driving at the current speed
void turnto(int degree, double ff = -1);  // 转向到某个角度，支持可选前馈增益参数
// EN: Turn to an absolute heading; optional feedforward gain `ff`
void move(int a, int c);  // 直接控制左右轮速度，会根据底盘电机数量自动调用
// EN: Set left/right wheel speeds directly (percent). The original says it adapts to the number of drive motors, but it only drives leftM/rightM
void tmove(int a, int c); // 转弯调用
// EN: Used for turning (speeds in rpm instead of percent)
void delay(double t);

