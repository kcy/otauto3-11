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
extern motor arm;
extern motor intake;
//基础配置页，对所有电机进行名称定义，方便后续调用
// 该配置页为自动赛配置页，所有电机均为自动赛使用
