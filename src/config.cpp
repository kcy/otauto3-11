#include "vex.h"

// Robot configuration code.
brain Brain;
inertial BrainInertial = inertial();
motor injL = motor(PORT8, false);
motor injR = motor(PORT11, true);
motor_group inj = motor_group(injL, injR);
motor leftM = motor(PORT7, true);
motor rightM = motor(PORT12, false);
motor arm = motor(PORT6, false);
motor intake = motor(PORT1, false);
distance frontdis = distance(PORT9);
distance backdis = distance(PORT2);
touchled TouchLED = touchled(PORT10);
controller Controller = controller();
