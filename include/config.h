using namespace vex;
extern brain Brain;
extern inertial BrainInertial;
extern controller Controller;
extern motor leftM;
extern distance backdis;
extern motor rightM;
extern distance frontdis;
extern touchled TouchLED;
extern motor injL;
extern motor injR;
extern motor_group inj; //此三行为电机组
// EN: These three lines (injL, injR, inj) define the launcher motor group
extern motor arm;
extern motor intake;
//基础配置页，对所有电机进行名称定义，方便后续调用
// EN: Basic configuration page: gives every motor a name so it is easy to use later
// 该配置页为自动赛配置页，所有电机均为自动赛使用
// EN: This is the autonomous configuration page; all motors here are used in autonomous
