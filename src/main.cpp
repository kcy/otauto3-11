#include "vex.h"
//----------------------------------------------------------------------------
//
//    Module:       main.cpp
//    Author:       shen qiang Mr不着急
//    Created:      {date}
//    Description:  IQ project
//
//----------------------------------------------------------------------------











//跑投距离
int run_shootdis=470;
//第111213包靠杆投和跑投后的半自动搜索fdown，根据fdcount次数去找可调节的函数
//队号信息
char teamNumber[] = "5333X";









// PID 参数与自动程序入口

extern void set_heading_pid(double p, double i, double d);
extern double tkp, tki, tkd;
extern PIDController headingPid;
bool ctchas = false;
bool autostate = false;
int autorun();
int chas();
int L4thred();
int cha, chc;
int L4 = 0;
int fdcount = 0;
void autoskill();
void autoqualification();
void autoinj();
void putL4();
bool autoinjstate = false;
int main() {
  // 配置底盘航向 PID 和转向 PID 参数
  // 直行航向 PID（用于 `run_dir_speed()` 的循迹修正）
  // set_heading_pid(0.5, 0.003, 0.5);
  set_heading_pid(0.4, 0.01, 1);
  // 转向 PID（用于 `turnto()`）
  tkp = 0.72;
  tki = 0.003;
  tkd = 0.9;
  // tkf = 0; // 前馈补偿，用于克服静摩擦并改善转向响应

  // 设置电机制动方式，提升停位稳定性
  arm.setBrake(brake);
  leftM.setBrake(brake);
  rightM.setBrake(brake);
  injL.setBrake(brake);
  injR.setBrake(brake);


  Brain.playNote(6, 6, 300);
  // 惯导校准：校准期间不允许进入自动
  // BrainInertial.calibrate();
  // while (BrainInertial.isCalibrating()) {
  //   wait(1, msec);
  // }
  BrainInertial.setRotation(0, degrees);
  Brain.playNote(6, 5, 300);
  // 机械结构初始化：先将机械臂和发射机构运行到基准位置
  TouchLED.setColor(green);
  arm.spin(forward,100,percent);
  wait(0.2, sec);
  arm.stop();
  inj.spin(forward,100,percent);
  wait(0.3, sec);
  while(inj.velocity(rpm)>3)
  {
    wait(10, msec);
  }
  inj.stop();
  arm.spin(forward,-100,percent);
  wait(0.2, sec);
  while(arm.velocity(rpm)<-2)
  {
    wait(10, msec);
  }
  arm.stop();
  wait(0.5, sec);
  arm.setPosition(0, degrees);
  // 启动自动程序、底盘控制和 L4 控制线程
  vex::task autor(autorun);
  vex::task chasiss(chas);
  vex::task L4thre(L4thred);
  while (1) {
    // 主循环负责制动状态、自动触发和手动输入
    if (arm.position(degrees) < 80) {
      arm.setBrake(brake);
    } else {
      arm.setBrake(hold);
    }

    // 由自动程序发出发射请求时执行一次发射动作
    if (autoinjstate == true) {
      autoinj();
      autoinjstate = false;
    }

    // 自动模式下允许按摇杆 A 退出自动程序
    if (autostate == true) {
      TouchLED.setColor(blue);
      if (Controller.AxisA.position() > 50) {
        TouchLED.setColor(red);
        autostate = false;
        task::stop(autor);
      }
    }

    if (autostate == false) {
      // 手动模式：处理自动启动、进气机构和发射控制
      if (TouchLED.pressing()) //自动及半自动准备
      {
        Brain.playNote(6, 5, 300);
        TouchLED.setColor(green);
        BrainInertial.setRotation(0,degrees);
        wait(0.2,seconds);
        arm.setVelocity(100, percent);
        arm.setTimeout(3, sec);
        arm.spinToPosition(0, degrees, true);
        L4=0;
        fdcount = 0;
        Brain.Timer.reset();
        while (TouchLED.pressing())
        {
          if(Brain.Timer.value() > 1.0) 
          {
            TouchLED.setColor(blue);
            while(TouchLED.pressing())
            {
              wait(10,msec);
            }
            vex::task autor(autorun);
            autostate = true;
          }
          wait(10, msec);
        }
      }
      if (Controller.ButtonLUp.pressing())
      {
        while (Controller.ButtonLUp.pressing())
        {
          wait(10, msec);
        }
        intake.spin(forward,100,percent);
      }
      else if (Controller.ButtonLDown.pressing())
      {
        while (Controller.ButtonLDown.pressing())
        {
          wait(10, msec);
          intake.spin(reverse,100,percent);
        }
        intake.stop();
      }
      if(Controller.ButtonEUp.pressing()&&arm.position(degrees)>200)
      {
        Brain.Timer.reset();
        while(Controller.ButtonEUp.pressing())
        {
          if(Brain.Timer.value()>2)
          {
            inj.spin(forward,-100,percent);
            wait(0.3, sec);
            // 终止所有程序：停止后台任务和所有运动部件
            Brain.programStop();
            return 0;
          }
          wait(10, msec);
        }
      }
      if ((Controller.ButtonFDown.pressing()
           /* 半自动发射，把自和前后注释符删掉即可启动 ||
              (Controller.AxisA.position() < -80 &&
               backdis.objectDistance(mm) < 25) */) &&
          L4 > 1 && arm.position(degrees) > 200) {
        // FDown：靠距离传感器选择近距离或远距离发射路径
        fdcount++;
        if (backdis.objectDistance(mm) < run_shootdis+10) {
          // 靠杆发射
          ctchas = true;
          move(-92, -100);
          while (backdis.objectDistance(mm) > 65) {
            wait(10, msec);
          }
          intake.stop();
          move(-100, -100);
          inj.spin(forward, -100, percent);
          wait(0.2, sec);
          BrainInertial.setRotation(-83, degrees);
          inj.spin(forward, 100, percent);
          intake.spin(forward, 100, percent);

          set_heading_pid(0.6, 0.00, 0.7);
          if (fdcount == 1) {
            run_dir_speed_dis(-46, 85, 25, false);
            set_heading_pid(0.4, 0.00, 0.7);
            run_dir_speed_dis(-46, 85, 13, false);
          } else if (fdcount == 2) {
            run_dir_speed_dis(-35, 85, 30, false);
            set_heading_pid(0.4, 0.00, 0.7);
            run_dir_speed_dis(-35, 85, 15, false);

          } else {
            run_dir_speed_dis(-30, 85, 30, false);
            set_heading_pid(0.4, 0.00, 0.7);
            run_dir_speed_dis(-30, 85, 22, false);

          }
          // run_dir_speed_dis(-53, 35, 10, false);

          inj.stop();
          ctchas = false;
        } else {
          // 远距离发射
          ctchas = true;
          move(-100, -100);
          while (backdis.objectDistance(mm) > run_shootdis) {
            wait(10, msec);
          }
          intake.stop();
          BrainInertial.setRotation(-87, degrees);
          inj.spin(forward, -100, percent);
          wait(0.20, sec);
          inj.spin(forward, 100, percent);
          intake.spin(forward, 100, percent);
          set_heading_pid(0.6, 0.00, 0.7);
          if (fdcount == 1) {
            run_dir_speed_dis(-40, 85, 29, false);
          } else if (fdcount == 2) {
            run_dir_speed_dis(-32, 85, 39, false);
          } else {
            run_dir_speed_dis(-25, 85, 44, false);
          }
          ctchas = false;
        }
        while (injL.velocity(rpm) > 3 || injR.velocity(rpm) > 3) {
          wait(10, msec);
        }
        inj.stop();
        while (Controller.ButtonFDown.pressing()) {
          wait(10, msec);
        }
      }
      if ((Controller.ButtonFUp.pressing()
      /*半自动发射，把自和前后注释符删掉即可启动||(Controller.AxisA.position()<-80&&backdis.objectDistance(mm)<25)*/)
      &&L4>1&&arm.position(degrees)>200) //控制发射
      {
        //常规弹射
        if(backdis.objectDistance(mm)<run_shootdis+10)
        {
          ctchas=true;
          move(-92,-100);
          while(backdis.objectDistance(mm)>65)
          {
            wait(5, msec);
          }
          intake.stop();
          inj.spin(forward,-100,percent);
          wait(0.20, sec);
          //wait(0.1,seconds);
          inj.spin(forward,100,percent);
          move(88,100);
          wait(0.55, sec);
          ctchas=false;
          intake.spin(forward,100,percent);
          //wait(0.3,seconds); 
          while(injL.velocity(rpm)>3||injR.velocity(rpm)>3)
          {
            wait(10, msec);
          }
          inj.stop();
          while (Controller.ButtonFUp.pressing())
          {
            wait(10, msec);
          }
        }
        else //远投
        {
          ctchas=true;
          move(-86,-100);
          while(backdis.objectDistance(mm)>run_shootdis)
          {
            wait(10, msec);
          }
          while(backdis.objectDistance(mm)>run_shootdis)
          {
            wait(5, msec);
          }
          move(-40,-100);
          intake.stop();
          //ctchas=true;
          inj.spin(forward,-100,percent);
          wait(0.18, sec);
          move(-10,100);
          inj.spin(forward,100,percent);
          wait(0.33, sec);
          move(84,100);
          wait(0.55, sec);
          ctchas=false;
          intake.spin(forward,100,percent);
          //wait(0.3,seconds); 
          while(injL.velocity(rpm)>3||injR.velocity(rpm)>3)
          {
            wait(10, msec);
          }
          inj.stop();
          while (Controller.ButtonFUp.pressing())
          {
            wait(10, msec);
          }
        }
        //
      }
    }
    wait(10, msec);
  }
}
// L4 机构控制线程：处理手动升降和自动模式切换
int L4thred() {
  while (1) {
    wait(30, msec);
    //显示所有信息
    Brain.Screen.setCursor(1, 1); // 设置光标位置
    
    Brain.Screen.clearScreen();//循环清屏
    Brain.Screen.setPenColor(purple);
    Brain.Screen.drawLine(0, 0, 159, 0);
    Brain.Screen.drawLine(0, 21, 159, 21);
    Brain.Screen.drawLine(0, 43, 159, 43);
    Brain.Screen.drawLine(0, 64, 159, 64);
    Brain.Screen.drawLine(0, 86, 159, 86);
    Brain.Screen.drawLine(0, 107, 159, 107);
    Brain.Screen.drawLine(0, 0, 0, 108);
    Brain.Screen.drawLine(80, 21, 80, 108);
    Brain.Screen.drawLine(159, 0, 159, 108);
    Brain.Screen.setFont(mono15);
    Brain.Screen.setPenColor(orange);
    Brain.Screen.printAt(50, 15, teamNumber);
    Brain.Screen.setPenColor(white);
    Brain.Screen.printAt(21, 36, "Angle");
    Brain.Screen.printAt(96, 36, "%6.1f", BrainInertial.rotation(degrees));
    Brain.Screen.setPenColor(white);
    Brain.Screen.printAt(16, 57, "Frontdis");
    Brain.Screen.printAt(103, 57, "%4.0f", frontdis.objectDistance(mm));
    Brain.Screen.setPenColor(white);
    Brain.Screen.printAt(20, 79, "Backdis");
    Brain.Screen.printAt(103, 79, "%4.0f", backdis.objectDistance(mm));
    Brain.Screen.setPenColor(white);
    Brain.Screen.printAt(16, 100, "Shootdis");
    Brain.Screen.printAt(103, 100, "%4d", run_shootdis);
  
    //显示所有信息
    if (autostate == false) {
      if (Controller.ButtonRUp.pressing()) {
        TouchLED.setColor(red);
        autostate = false;
        leftM.setStopping(brake);
        rightM.setStopping(brake);
        //task::stop(autorun);
        while (Controller.ButtonRUp.pressing()) {
          wait(10, msec);
        }
        L4++;
        putL4();
        arm.spin(forward,-100,percent);
        if (L4 < 3) 
        {
         wait(400, msec);
          while(fabs(arm.velocity(rpm))>3)
          {
            wait(10, msec);
          }
          arm.setVelocity(0, percent);
        } 
        else {
          arm.setVelocity(100, percent);
          arm.spinToPosition(270, degrees, false);
        }
      } 
      else if (Controller.ButtonRDown.pressing()) {
        while (Controller.ButtonRDown.pressing()) {
          wait(10, msec);
        }
        L4 = 2;
        arm.spin(forward,-100,percent);
         wait(400, msec);
          while(fabs(arm.velocity(rpm))>8)
          {
            wait(10, msec);
          }
          arm.setVelocity(0, percent);
      }
    }
  }
}
// 自动程序线程：根据启动方式执行半自动或全自动流程
int autorun() {
  while (1) {
    if (autostate == true && Controller.ButtonFUp.pressing()) {
      autoqualification();
      // 半自动程序入口
      // L4 = 3;
      TouchLED.setColor(red);
      autostate = false;
      break;
    } else if (autostate == true &&
               (TouchLED.pressing() || Controller.ButtonEDown.pressing())) {
      // 全自动程序入口
      autoskill();
      autostate = false;
      break;
      
    }
    wait(10, msec);
  }
  return 0;
}
// 底盘控制线程：手动模式下读取摇杆，自动发射时暂停驾驶输出
int chas() {
  while (1) {
    if (autostate == false) {
      if (ctchas == false) {
        if (Controller.AxisA.position() > 10 ||
            Controller.AxisA.position() < -10) {
          cha = Controller.AxisA.position();
        } else {
          cha = 0;
        }
        if (Controller.AxisC.position() > 13 ||
            Controller.AxisC.position() < -13) {
          chc = Controller.AxisC.position();
        } else {
          chc = 0;
        }
        if (cha > 70 && BrainInertial.rotation(degrees) > -105 &&
            BrainInertial.rotation(degrees) < -75) {
          move(cha + 0.5 * chc - 13, cha - 0.5 * chc);
        } else if (cha < -70 && BrainInertial.rotation(degrees) > -105 &&
                   BrainInertial.rotation(degrees) < -75) {
          move(cha + 0.5 * chc +13, cha - 0.5 * chc);
        } else {
          move(cha + 0.6 * chc, cha - 0.6 * chc);
        }
      }
    } 
    else {
      // 自动模式下，摇杆大幅推动时立即停止自动程序
      if (Controller.AxisA.position() > 90 ||
          Controller.AxisA.position() < -90) 
      {
        TouchLED.setColor(red);
        autostate = false;
        task::stop(autorun);

        leftM.setBrake(brake);
        rightM.setBrake(brake);
      } 
    }
    wait(10, msec);
  }
  return 0;
}
void autoqualification() {
  // 资格赛流程：初始化机构后依次处理场上的目标
  set_heading_pid(0.4, 0.01, 1);
  leftM.setBrake(brake);
  rightM.setBrake(brake);
  TouchLED.setColor(red);
  BrainInertial.resetRotation();
  BrainInertial.setRotation(0, degrees);
  intake.spin(forward, 100, percent);
  arm.spin(forward, 30, percent);
  run_dir_speed_dis(-37, 30, 5, false);
  run_dir_speed_time(-37, 34, 0.7, true);
  putL4();
  L4 = 1;
  arm.setVelocity(100, percent);
  arm.spinToPosition(0, degrees, false);
  // run_dir_speed_time(-28, -20, 0.2, true);
  // rightM.setVelocity(40, percent);
  rightM.setPosition(0, degrees);
  rightM.setVelocity(70, percent);
  rightM.spinToPosition(-73, degrees, true);

  // 转向并收集地上的第一个目标
  intake.spin(forward, 100, percent);
  run_dir_speed_dis(3, 40, 3.5, true);
  wait(0.3, seconds);
  rightM.setPosition(0, degrees);
  move(-50, -50);
  while (rightM.position(degrees) > -80) {
    wait(2, msec);
  }
  //wait(1,seconds);
  //arm.spin(forward,80,percent);
  //run_dir_speed_time(-25,22,1.3,false);
  //turnto(-28);
  //move(-25,25);
  leftM.setVelocity(-26, rpm);
  rightM.setVelocity(26, rpm);
  while (frontdis.objectDistance(mm) > 260) {
    wait(2, msec);
  }
  while (frontdis.objectDistance(mm) < 260) {
    wait(2, msec);
  }
  leftM.stop(hold);
  rightM.stop(hold);
  wait(0.3, seconds);
  arm.spin(forward, 100, percent);
  // wait(0.2, seconds);
  run_dir_speed_time(-30, 30, 0.4, false);
  putL4();
  L4 = 2;
  arm.setVelocity(100, percent);
  arm.spinToPosition(0, degrees, false);
  /*rightM.setPosition(0,degrees);
  move(-60,-60);
  while(rightM.position(degrees)>-50)
  {
    wait(8,msec);
  }*/
  move(0, -60);
  while (frontdis.objectDistance(mm) > 260) {
    wait(2, msec);
  }
  while (frontdis.objectDistance(mm) < 260) {
    wait(2, msec);
  }
  rightM.setPosition(0, degrees);
  while (rightM.position(degrees) > -28) {
    wait(8, msec);
  }
  leftM.stop(hold);
  rightM.stop(hold);
  wait(0.1, seconds);
  //turnto(30);//转向第二个黄包
  run_dir_speed_dis(28,60,10,true);
  wait(0.3,seconds);
  rightM.setPosition(0,degrees);
  move(-75,-75);
  while(rightM.position(degrees)>-85)
  {
    wait(8,msec);
  }
  //move(-25,25);
  leftM.setVelocity(-45, rpm);
  rightM.setVelocity(45, rpm);
  while(BrainInertial.rotation(degrees)>0)
  {
    wait(2,msec);
  }
  while(frontdis.objectDistance(mm)>360)
  {
    wait(2,msec);
  }
  
  //turnto(-30);
  //intake.setVelocity(0,percent);
  //leftM.spin(forward,-70,percent);
  //while(BrainInertial.rotation(degrees)>-7)
  //{
  //  wait(8,msec);
  //}
  move(0, 0);
  wait(0.3, seconds);
  arm.spin(forward,100,percent);
  run_dir_speed_time(-30,35,0.3,false);
  putL4();
  arm.setVelocity(100,percent);
  arm.spinToPosition(270,degrees,false);
  L4=3;

  //turnto(-50);
  rightM.setVelocity(0,rpm);
  rightM.stop(hold);
  //rightM.spinFor(-10,degrees,false);
  leftM.spin(forward,-90,rpm);
  while (BrainInertial.rotation(degrees)>-95)
  {
    wait(8,msec);
  }
  rightM.stop(hold);
  leftM.stop(hold);
  /*
  // 这里写联赛自动程序
  /**/ 
  TouchLED.setColor(red);
  wait(0.1, seconds);
  leftM.setBrake(brake);
  rightM.setBrake(brake);
}
// 技能赛自动程序
void autoskill() {
  // 起始动作：完成第一组机构操作并移动到发射位置
  // set_heading_pid(0.3, 0.001, 0.7); // 后退 PID
  autoqualification();
  // wait(0.3, seconds);
  run_dir_speed_dis(-117,75,14.5,true);
  //wait(0.1,seconds);
  leftM.setVelocity(90,percent);
  rightM.setVelocity(90,percent);
  leftM.spinFor(75,degrees,false);
  rightM.spinFor(-75,degrees,true);
  leftM.setBrake(brake);
  rightM.setBrake(brake);
  rightM.stop(brake);
  move(-88, -95);
  wait(1.3, seconds);
  autoinjstate = true;
  wait(0.2, seconds);
  set_heading_pid(0.4, 0.00, 0.7);
  run_dir_speed_dis(-86, 80, 30, true);
  wait(0.1, seconds);
  move(-87, -100);
  while (backdis.objectDistance(mm) > 60)
  {
    wait(2, msec);
  }
  autoinjstate = true;
  wait(0.2, seconds);
  BrainInertial.setRotation(-83, degrees);
  for (int i = 0; i < 3; i++) {
    // run_dir_speed_dis(-92, 90, 30, false);
    // run_dir_speed_time(-92, 90, 0.6, false);
    move(87, 100);
    wait(2.05, seconds);
    move(-85, -100);
    while (backdis.objectDistance(mm) > 60)
    {
      wait(2, msec);
    }
    while (backdis.objectDistance(mm) > 60)
    {
      wait(2, msec);
    }
    // run_dir_speed_time(-84, -92, 2.1, false);
    autoinjstate = true;
    wait(0.2, seconds);
    // BrainInertial.setRotation(-83, degrees);
  }
  move(87, 100);
    wait(2.05, seconds);
    move(-85, -100);
    while (backdis.objectDistance(mm) > 60)
    {
      wait(2, msec);
    }
    while (backdis.objectDistance(mm) > 60)
    {
      wait(2, msec);
    }
    // run_dir_speed_time(-84, -92, 2.1, false);
    wait(0.2, seconds);
    autoinjstate = true;
    wait(0.2, seconds);
  BrainInertial.setRotation(-83, degrees);
  set_heading_pid(0.6, 0.00, 0.7);
  run_dir_speed_dis(-51, 85, 25, false);
  set_heading_pid(0.4, 0.00, 0.7);
  run_dir_speed_dis(-51, 85, 13, false);

  // 移动到蓝色目标区域并完成连续发射
  run_dir_speed_dis(-70, -70, 8.5, false);
  move(-95, -86);
  wait(1.4, seconds);
  //BrainInertial.setRotation(-83, degrees);
  autoinjstate = true;
  wait(0.20, seconds);
  set_heading_pid(0.4, 0.00, 0.8);
  //开始打4个蓝
  run_dir_speed_dis(-95, 85, 14, false);
  run_dir_speed_dis(-45, 42, 8.6, false);
  run_dir_speed_time(-88, 60, 1.65, false);
  //run_dir_speed_time(-90, 63, 1.2, false);
  //BrainInertial.setRotation(-90, degrees);
 
  // 拿到第一个蓝
  
  // 往返收集剩余目标
  for (int i = 0; i < 3; i++) {
    set_heading_pid(0.4, 0.002, 0.8);
    run_dir_speed_dis(-196, -45, 10, false);
    set_heading_pid(0.3, 0.001, 0.6);
    while(frontdis.objectDistance(mm) <510)
    {
      run_dir_speed(-196, -45);
    }
    move(-45, -45);
    autoinjstate = true;
    wait(0.15, seconds);
    set_heading_pid(0.52, 0.002, 0.5);
    while(frontdis.objectDistance(mm) > 460)
    {
      run_dir_speed(-160, 53);
    }
    set_heading_pid(0.4, 0.002, 0.7);
    
    run_dir_speed_time(-94, 45, 1.5, false);
    // wait(2000, seconds);
  }
  set_heading_pid(0.4, 0.002, 0.8);
    run_dir_speed_dis(-196, -45, 10, false);
    set_heading_pid(0.3, 0.001, 0.6);
    while(frontdis.objectDistance(mm) <510)
    {
      run_dir_speed(-196, -45);
    }
    move(-45, -45);
    autoinjstate = true;
    wait(0.15, seconds);
  intake.stop();
}
// 资格赛自动程序

// 执行一次发射机构往复动作
void autoinj() {
  inj.spin(forward, -100, percent);
  wait(0.3, sec);
  inj.spin(forward, 100, percent);
  wait(0.3, sec);
  while (inj.velocity(rpm) > 2) {
    wait(10, msec);
  }
  inj.stop();
}
// 将机械臂抬到 L4 放置位置，并等待机构稳定
void putL4() {
  arm.spin(forward, 100, percent);
  while (arm.position(degrees) < 150) {
    wait(10, msec);
  }
  while (arm.velocity(rpm) > 7) {
    wait(10, msec);
  }
  wait(100, msec);
}