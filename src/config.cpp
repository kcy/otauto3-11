#include "vex.h"

// Robot configuration code.
brain Brain;
inertial BrainInertial = inertial();
motor injL = motor(PORT8, false);
motor injR = motor(PORT12, true);
motor_group inj = motor_group(injL, injR);
motor leftM = motor(PORT7, true);
motor rightM = motor(PORT10, false);
motor arm = motor(PORT1, false);
motor intake = motor(PORT6, false);
distance frontdis = distance(PORT4);
distance backdis = distance(PORT5);
touchled TouchLED = touchled(PORT2);
controller Controller = controller();
