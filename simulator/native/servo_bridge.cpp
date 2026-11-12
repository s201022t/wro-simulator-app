#include "servo_lib.h"
#include <Arduino.h>
static int angles[4]={-180,50,25,45};
void servoSetup(){} bool startServoMoveByPin(int p,int a,int){if(p<0||p>3)return false;angles[p]=a;return true;}
void updateServoTasks(){} void servoDelay(unsigned long ms){delay(ms);} void cancelServoTasks(){} bool servoTasksBusy(){return false;} void waitServoTasks(){}
void writeBothServos(int a,int b){angles[0]=a;angles[1]=b;} bool resetServoToAngleByPin(int p,int a){return startServoMoveByPin(p,a,255);}
void resetGeekServosToStartAngle(){} int getServoDirByPin(int){return 1;} int applyServoDirByPin(int,int a){return a;} void printServoSettings(){}
void catchArm(int p,int s,bool w){startServoMoveByPin(1,p,s);if(w)servoDelay(100);} void liftArm(int p,int s,bool w){startServoMoveByPin(0,p,s);if(w)servoDelay(100);}
void backArm(int p,int s,bool w){startServoMoveByPin(2,p,s);if(w)servoDelay(100);} void circleArm(int p,int s,bool w){startServoMoveByPin(3,p,s);if(w)servoDelay(100);}
void front(int,int,int,bool){}
