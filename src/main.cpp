#include "vex.h"
//----------------------------------------------------------------------------
//                                                                            
//    Module:       main.cpp                                                  
//    Author:       shen qiang                                                
//    Created:      {date}                                                    
//    Description:  IQ project                                                
//                                                                            
//----------------------------------------------------------------------------

int side = 0; // 默认蓝色方蓝色
int ctclaw = 1;
int claw = 1, upclaw = 0;
int ctchas = 1, autostate = 0;

int main()
{
  Pneumatic1.pumpOn();
  arm.setStopping(brake);
  leftM.setStopping(brake);
  rightM.setStopping(brake);      
  arm.setPosition(0, degrees);
  UPDOWN.setStopping(brake);
  UPDOWN.setPosition(0, degrees);
  arm.setPosition(0, degrees);
  vex::task autor(autorun);
  vex::task ctxianwei(xianwei);
  Pneumatic1.pumpOn();
  TouchLED7.setColor(yellow);
  beam(0);
  while (1)
  {
    wait(5, msec);
    if (arm.position(degrees) < 50)
    {
      if (leftdistance.objectDistance(mm) < 75 && claw == 1)
      {
        Pneumatic1.retract(cylinder1);
      }
      else
      {
        Pneumatic1.extend(cylinder1);
      }
      if (rightdistance.objectDistance(mm) < 75 && claw == 1)
      {
        Pneumatic8.retract(cylinder2);
      }
      else
      {
        Pneumatic8.extend(cylinder2);
      }
    }
    else
    {
      if (claw == 1)
      {
        Pneumatic1.retract(cylinder1);
      }
      else
      {
        Pneumatic1.extend(cylinder1);
      }
      if (claw == 1)
      {
        Pneumatic8.retract(cylinder2);
      }
      else
      {
        Pneumatic8.extend(cylinder2);
      }
    }
    if (TouchLED7.pressing() && autostate != 3)
    {
      Brain.Timer.reset();
      while (TouchLED7.pressing())
      {
        wait(5, msec);
        if (Brain.Timer.value() > 1)
        {
          Brain.playNote(3, 7, 1000);
          TouchLED7.setColor(red);
          Pneumatic1.pumpOff();
          BrainInertial.calibrate();
          while (BrainInertial.isCalibrating())
          {
            wait(1, msec);
          }
          TouchLED7.setColor(green);
          BrainInertial.setRotation(0, degrees);
          autostate = 3;
        }
      }
    }
    if (Controller.ButtonFUp.pressing() && autostate != 3)
    {
      Brain.Timer.reset();
      while (Controller.ButtonFUp.pressing())
      {
        wait(5, msec);
        if (Brain.Timer.value() > 1)
        {
          Brain.playNote(3, 7, 1000);
          TouchLED7.setColor(red);
          Pneumatic1.pumpOff();
          BrainInertial.calibrate();
          while (BrainInertial.isCalibrating())
          {
            wait(1, msec);
          }
          TouchLED7.setColor(green);
          BrainInertial.setRotation(0, degrees);
          autostate = 3;
        }
      }
    }
    if (autostate == 3)
    {
      if (TouchLED7.pressing() == 1 || Controller.ButtonFUp.pressing())
      {
        autostate = 2;
      }
    } 
  }
}

int autorun()
{
  while (1)
  {
    wait(1, msec);
    if (autostate == 2)
    {
      BrainInertial.setRotation(0, degrees);
      Pneumatic1.pumpOn();
      Optical9.setLight(ledState::on);
      run_dir_speed_dis(50, 37, 4, false);
      run_dir_speed_dis(50, 60, 3, false);//拿蓝
      run_dir_speed_dis(-45, 50, 8.3, false);
      run_dir_speed_dis(-8, 40, 4, false);
      run_dir_speed_dis(-8, 70, 6, false);
      run_dir_speed_dis(55, 40, 3, false);
      run_dir_speed_dis(55, 70, 4, false);//拿红
      int mid = 20;
      Brain.Screen.print("%d", mid);
      run_dir_speed_dis(87, 40, 4, false);
      run_dir_speed_dis(87, 70, 8.7, true);//推黄
      wait(0.2,seconds);
      run_dir_speed_dis(87, -30, 1.9, true);//退一点
      turnto(165);
      run_dir_speed_time(165, -70, 1, false);
      run_dir_speed_time(165, 60, 0.2, false);
      run_dir_speed_time(165, -60, 0.3, false);
      wait(0.1, seconds);
      Pneumatic1.retract(cylinder2);
      beam(1);
      beam(1);
      wait(0.2, seconds);
      BrainInertial.setRotation(0, degrees);
      run_dir_speed_dis(0, 80, 6, false);
      run_dir_speed_time(0, 60, 0.7, true);//预装对位
      int predeg = BrainInertial.rotation(degrees);
      run_dir_speed_dis(predeg, -40, 0.6, true);
      claw = 0;
      wait(0.1, seconds);
      claw = 1;
      wait(0.2, seconds);
      arm.setVelocity(90, percent);
      arm.spinToPosition(450, degrees, false);
      wait(0.5, seconds);
      run_dir_speed_time(predeg, 50, 0.4, false);
      beam(0);
      beam(0);
      run_dir_speed_time(predeg, 50, 0.2, false);//预装
      run_dir_speed_dis(predeg, -50, 3.5, false);
      beam(1);
      beam(1);
      run_dir_speed_dis(predeg, -40, 1, true);
      UPDOWN.spinToPosition(55, degrees, false);
      wait(0.2, seconds);
      arm.spinToPosition(540, degrees, false);
      wait(0.4, seconds);
      claw = 0;//反扣一层
      claw = 0;
      wait(0.3, seconds);
      arm.spinToPosition(0, degrees, false);
      turnto(-90);
      arm.setStopping(brake);
      claw = 1;
      wait(0.2,seconds);
      if (predeg > 0)
      {
        run_dir_speed_dis(-90, 60, 7, true);
      }
      else
      {
        run_dir_speed_dis(-90, 60, 8.2, true);
      }
      arm.spin(forward, 90, percent);
      while(arm.position(degrees)<300)
      {
        wait(1,msec);
      }
      arm.setVelocity(40, percent);
      arm.spinToPosition(500, degrees, true);
      claw = 0;//反扣二层
      claw = 0;
      wait(0.2, seconds);
      arm.setVelocity(90, percent);
      arm.spinToPosition(0, degrees, false);
      UPDOWN.spinToPosition(700, degrees, false);
      if (predeg > 0)
      {
        run_dir_speed_dis(-90, -60, 5.9, true);
      }
      else
      {
        run_dir_speed_dis(-90, -60, 5.9, true);
      }
      turnto(-180);//转向高桩
      run_dir_speed_time(-180, -60, 1.4, false);
      Optical9.setLight(ledState::on);
      Optical9.setLightPower(100, percent);
      UPDOWN.spinToPosition(580, degrees, true);
      beam(0);
      beam(0);//释放高桩
      while (Optical9.brightness() > mid)
      {
        run_dir_speed(-180, 40);
      }
      run_dir_speed_dis(-180, 40, 2, false);
      UPDOWN.spinToPosition(300, degrees, false);
      turnto(-90);
      leftM.setPosition(0, degrees);
      while (Optical9.brightness() > mid)
      {
        run_dir_speed(-90, 30);
      }
      arm.spinToPosition(0, degrees, false);
      claw = 1;
      if (leftM.position(degrees) < 350)
      {
        run_dir_speed_dis(-88, 60, 7.5, false);
      }
      else
      {
        run_dir_speed_dis(-88, 60, 1.9, false);
      }
      run_dir_speed_dis(-60, 50, 3.5, false);
      run_dir_speed_dis(-60, 80, 13.5, false);//拿俩黄
      run_dir_speed_time(-90, 80, 0.8, true);
      run_dir_speed_dis(-90, -60, 1.2, true);
      leftM.setVelocity(40, percent);
      leftM.spinFor(forward, -220, degrees, true);
      wait(0.1, seconds);
      int beam2deg = BrainInertial.rotation(degrees);
      run_dir_speed_dis(beam2deg - 12, 55, 3, false);
      UPDOWN.spinToPosition(45, degrees, false);//平行四边形降低
      leftM.setPosition(0,degrees);
      while(leftM.position(degrees)<200)
      {
        move(45,60);
      }
      //run_dir_speed_dis(-170, 60, 4, true);
      run_dir_speed_dis(-179, 40, 3.7, true);
      rightM.spin(forward, 30, percent);
      while (BrainInertial.rotation(degrees) > -180)
      {
        wait(2, msec);
      }
      run_dir_speed_dis(-181, -30, 2.5, false);
      while (Optical9.brightness() > mid)
      {
        run_dir_speed(-181, -40);
      }
      run_dir_speed_dis(-181, -60, 2, false);
      run_dir_speed_dis(-181, -60, 2.2, false);
      arm.spinToPosition(400, degrees, false);
      run_dir_speed_dis(-181, -60, 8.4, false);
      UPDOWN.spinToPosition(0, degrees, false);
      while (Optical9.brightness() > mid)
      {
        run_dir_speed(-181, -35);
      }
      run_dir_speed_dis(-181, -60, 1.2, false);
      beam(1);
      beam(1);
      run_dir_speed_dis(-182, -50, 1, true);
      UPDOWN.spinToPosition(55, degrees, false);
      wait(0.2, seconds);
      arm.setVelocity(90, percent);
      arm.spinToPosition(540, degrees, false);//扣完第二个一层
      wait(0.5, seconds);
      claw = 0;
      claw = 0;
      wait(0.2, seconds);
      arm.spinToPosition(0, degrees, false);
      rightM.spin(forward, 40, percent);//单轮转向红蓝色
      while (BrainInertial.rotation(degrees) > -218)
      {
        wait(1, msec);
      }
      move(0, 0);
      wait(0.1, seconds);
      claw = 1;
      run_dir_speed_dis(-230, 60, 11, true);
      arm.setVelocity(80, percent);
      arm.spinToPosition(400, degrees, false);
      run_dir_speed_dis(-245, -60, 4, true);
      wait(0.1, seconds);
      arm.setVelocity(50, percent);
      arm.spinToPosition(540, degrees, false);
      wait(0.1, seconds);
      turnto(-360);
      claw = 0;
      claw = 0;
      wait(0.2, seconds);
      arm.setVelocity(70, percent);
      arm.spinToPosition(0, degrees, true);//扣完2双层
      wait(0.2, seconds);
      claw = 1;
      run_dir_speed_dis(-360, 80, 5, true);//做91的双色底
      arm.setVelocity(100, percent);
      arm.spinToPosition(170, degrees, false);
      wait(0.1, seconds);
      run_dir_speed_dis(-360, 35, 5.5, true);
      arm.spinToPosition(0, degrees, false);
      wait(0.2, seconds);
      claw = 0;
      run_dir_speed_time(-360, 30, 0.3, true);
      claw = 1;//抓住双色底座
      run_dir_speed_dis(-360, -60, 3, false);
      claw = 0;
      run_dir_speed_dis(-360, -60, 4, true);
      turnto(-530);
      UPDOWN.setVelocity(100, percent);
      UPDOWN.spinToPosition(430, degrees, true);
      run_dir_speed_dis(-530, -35, 6.5, true);
      UPDOWN.setVelocity(60, percent);
      UPDOWN.spinToPosition(250, degrees, true);
      beam(0);
      wait(0.2, seconds);
      run_dir_speed_dis(-530, 70, 1.3, true);
      turnto(-630);//做最后一个同色黄底加分区
      claw = 1;
      run_dir_speed_dis(-630, 70, 15, true);
      arm.setVelocity(100, percent);
      arm.spinToPosition(170, degrees, true);
      run_dir_speed_dis(-630, 50, 4.5, false);
      arm.spinToPosition(0, degrees, false);
      run_dir_speed_dis(-630, 40, 2, true);
      claw = 0;
      run_dir_speed_dis(-480, 15, 4, false);
      claw=1;
      run_dir_speed_dis(-485, 50, 5.2, true);
      claw=0;
      wait(0.1,seconds);
      move(-60,-60);
      wait(0.5,seconds);
      move(0,0);
      //run_dir_speed_dis(-633, -40, 2, true);
      /*run_dir_speed_dis(-530, 70, 2, true);
      UPDOWN.spinToPosition(400, degrees, false);
      run_dir_speed_time(-575, -30, 0.8, true);
      run_dir_speed_time(-575, -60, 1, true);
      /*
      turnto(-270);//转向第二个底座黄pin
      claw=0;
      claw=0;
      wait(0.2,seconds);
      arm.setVelocity(90,percent);
      arm.spinToPosition(0,degrees,true);//扣完2双层
      claw=1;
      wait(0.1,seconds);
      run_dir_speed_dis(-258,30,7,true);
      run_dir_speed_dis(-258,-30,2,false);
      claw=0;
      run_dir_speed_dis(-258,-50,3,true);
      turnto(-447);
      UPDOWN.spinToPosition(430,degrees,true);
      run_dir_speed_dis(-447,-35,4.8,true);
      UPDOWN.spinToPosition(200,degrees,true);
      beam(0);
      wait(0.2,seconds);
      run_dir_speed_dis(-447,60,2,true);//最后一个双色黄
      turnto(-360);
      claw=1;
      run_dir_speed_dis(-360,80,8,true);
      wait(0.1,seconds);
      arm.setVelocity(100,percent);
      arm.spinToPosition(180,degrees,false);
      wait(0.1,seconds);
      run_dir_speed_dis(-360,40,4,true);
      arm.spinToPosition(0,degrees,false);
      wait(0.2,seconds);
      claw=0;
      run_dir_speed_dis(-360,-40,2,true);
      /*while(Optical9.hue()<150)
      {
        run_dir_speed(-360,40);
      }
      arm.spinToPosition(170,degrees,false);
      turnto(-415);
      UPDOWN.spinToPosition(600,degrees,false);
      run_dir_speed_dis(-415,30,9,true);
      arm.spinToPosition(0,degrees,false);
      wait(0.5,seconds);
      claw=0;
      run_dir_speed_dis(-415,-30,3,true);
      turnto(-245);
      //leftM.spinFor(forward,-180,degrees,false);
      //rightM.spinFor(forward,180,degrees,true);
      /*upclaw=1;
      Pneumatic1.retract(cylinder2);
      upclaw=1;
      Pneumatic1.retract(cylinder2);
      delay(0.1);
      UPDOWN.spinToPosition(60,degrees,true);
      arm.setVelocity(90,percent);
      arm.spinToPosition(550,degrees,false);
      run_dir_speed_dis(135,40,4,true);
      delay(0.3);
      claw=0;
      Pneumatic1.extend(cylinder1);
      delay(0.1);
      arm.setVelocity(80,percent);
      arm.spinToPosition(0,degrees,false);
      delay(0.3);
      turnto(85);
      claw=0;
      run_dir_speed_dis(85,40,3,true);
      claw=0;*/
      autostate = 0;
      TouchLED7.setColor(yellow);
      ctchas = 1;
    }
  }
}