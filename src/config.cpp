#include "vex.h"
// Robot configuration code.
brain Brain;
inertial BrainInertial = inertial();
motor UPDOWNMotorA = motor(PORT5, false);
motor UPDOWNMotorB = motor(PORT11, true);
motor_group UPDOWN = motor_group(UPDOWNMotorA, UPDOWNMotorB);

controller Controller = controller();
motor armMotorA = motor(PORT4, false);
motor armMotorB = motor(PORT10, true);
motor_group arm = motor_group(armMotorA, armMotorB);

pneumatic Pneumatic1 = pneumatic(PORT1);
motor leftM = motor(PORT6, true);
motor rightM = motor(PORT12, false);
distance leftdistance = distance(PORT2);
distance rightdistance = distance(PORT3);
touchled TouchLED7 = touchled(PORT7);
pneumatic Pneumatic8 = pneumatic(PORT8);
optical Optical9 = optical(PORT9);