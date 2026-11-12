#include "motion_lib.h"
#include "servo_lib.h"
#include "pi_color_lib.h"
#include "extra.h"
#include "robot_app.h"
#include <Arduino.h>
#include <exception>
#include <iostream>
#include <string>
void section_A(); void section_B(); void section_C();

int main(int argc,char**argv){
 std::string program="demo";bool quiet=false;for(int i=1;i<argc;i++){std::string a=argv[i];if(a=="--program"&&i+1<argc)program=argv[++i];else if(a=="--quiet")quiet=true;}
 (void)quiet;sim::reset();
 try{servoSetup();motionSetup();piColorSetup();resetYawDeg();
  if(program=="demo"){tank(300);spin(90,220);tank(300);tankOff();}
  else if(program=="line"){tline(500,180,1);}
  else if(program=="reverse"){tank(-500);}
  else if(program=="spin-left"){spin(-90,220);}
  else if(program=="servo"){resetGeekServosToStartAngle();catchArm(100,80,0);liftArm(75,120,0);waitServoTasks();backArm(100,100,1);circleArm(100,100,1);}
  else if(program=="color"){if(!requestColorRead())return 3;}
  else if(program=="current"){runCurrentProgram();tankOff();}
  else if(program=="section-a"){section_A();tankOff();}
  else if(program=="section-b"){section_B();tankOff();}
  else if(program=="section-c"){section_C();tankOff();}
  else if(program.rfind("extra-",0)==0){int n=std::stoi(program.substr(6));void(*f[])()={extra_1,extra_2,extra_3,extra_4,extra_5,extra_6,extra_7,extra_8,extra_9,extra_10};if(n<1||n>10)return 2;f[n-1]();tankOff();}
  else {std::cerr<<"Unknown program: "<<program<<"\n";return 2;}
  sim::emitState(true);std::cout<<"{\"type\":\"complete\",\"program\":\""<<program<<"\"}"<<std::endl;
 }catch(const std::exception&e){std::cout<<"{\"type\":\"error\",\"message\":\""<<e.what()<<"\"}"<<std::endl;return 1;}return 0;
}
