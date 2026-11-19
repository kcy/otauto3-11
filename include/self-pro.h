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
void run_dir_speed_dis(double degree, double speed, double dis, bool st = true);
// 沿指定方向走指定距离。dis 单位为英寸；若沿用旧代码，使用 dis *
// OLD_DIS_TO_INCH 进行转换
void run_dir_speed_time(double degree, double speed, double time,
                        bool st = true);
// 沿着x方向以x速度走多长时间，二级封装，带时间参数，st为true时结束后会停下来，false时结束后继续保持当前速度
void turnto(int degree, double ff = -1);  // 转向到某个角度，支持可选前馈增益参数
void move(int a, int c);  // 直接控制左右轮速度，会根据底盘电机数量自动调用
void tmove(int a, int c); // 转弯调用
void delay(double t);

