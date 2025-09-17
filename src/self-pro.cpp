#include "vex.h"
double kp=0.7,ki=0.001,kd=0.2,err,integ,deri,lasterr,pid;
double tkp=0.75,tki=0.005,tkd=0.4,terr,tinteg,tderi,tlasterr,tpid;
int upmini=2;
int cha,chc;int armstate=1;
int breakauto=0;
void run_dir_speed(double degree,double speed)
{
  wait(10,msec);
  err=degree-BrainInertial.rotation(degrees);
  if (fabs(err)>15||fabs(err)<2)
  {
    integ=0;
  }
  else 
  {
    integ+=err;
  }
  deri=err-lasterr;
  pid = err*kp+integ*ki+deri*kd;
  move(speed+pid,speed-pid);
  lasterr=err;
}
void run_dir_speed_dis(double degree,double speed,double dis,bool st)
{
  integ=0;deri=0;lasterr=0;
  leftM.setPosition(0,degrees);
  rightM.setPosition(0,degrees);
  while(fabs(leftM.position(degrees))+fabs(rightM.position(degrees))<dis*100)
  {
    run_dir_speed(degree,speed);
    if(breakauto==1)
    {
      break;
    }
  }
  if (st==true)
  {
    move(0,0);
  }
}
void run_dir_speed_time(double degree,double speed,double time,bool st)
{
  Brain.Timer.reset();
  while(Brain.Timer.value()<time)
  {
    run_dir_speed(degree,speed);
    if(breakauto==1)
    {
      break;
    }
  }
  if (st==true)
  {
    move(0,0);
  }
}
void turnto(int degree)
{
  int count=0;
  while(1)
  {
    if(breakauto==1)
    {
      break;
    }
    wait(10,msec);
    terr=degree-BrainInertial.rotation(degrees);
    if (fabs(terr)>10||fabs(terr)<4)
    {
      tinteg=0;
    }
    else 
    {
      tinteg+=terr;
    }
      if(fabs(terr)<4)
    {
      count+=1;
      if (count>15)
      {
        break;
      }
    }
    tderi=terr-tlasterr;
    tpid = terr*tkp+tinteg*tki+tderi*tkd;
    move(tpid,-tpid);
    tlasterr=terr;
  }
  move(0,0);
}
void move(int a,int c)
{
  leftM.spin(forward,a,percent);
  rightM.spin(forward,c,percent);
}
void delay(double t)
{
  Brain.Timer.reset();
  while(Brain.Timer.value()<t)
  {
    wait(1,msec);
    if(breakauto==1)
    {
      break;
    }
  }
}
void beam(int st)
{
  if(st==1)
  {
    Pneumatic1.retract(cylinder2);
  }
  else 
  {
    Pneumatic1.extend(cylinder2);
  }
}
int xianwei()
{
  while (1)
  {
    wait(0.01,seconds);
    if(UPDOWN.position(degrees)<420||UPDOWN.position(degrees)>680)
    {
      Pneumatic8.extend(cylinder1);
    }
    else 
    {
      Pneumatic8.retract(cylinder1);
    }
  }
}
