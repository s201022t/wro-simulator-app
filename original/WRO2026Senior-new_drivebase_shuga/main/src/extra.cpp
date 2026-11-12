#include "motion_lib.h"
#include "servo_lib.h"
#include "pi_color_lib.h"
#include "move_function.h"
#include "extra.h"
#include <Wire.h>
#include <math.h>
#include <Adafruit_NeoPixel.h>
#include <Arduino.h>

void extra_1() {
robot_start();
  tlineR(90, 140, 0, 100);
  tank(73, 80);
  requestColorRead();


String color;

  move_mosaic2_3('G', 'G');
  tline(1250, 255, 0);
  tank(73);
  spin(-90, 255);
  tline(310, 200, 0);
  tank(73);
  spin(-90, 255);
    pivot(21, 210, R);
  pivot(-18, 210, L);
  servoDelay(50);
  startServoMoveByPin(1, 0, 200);    //開ける
  startServoMoveByPin(0, 30, 255);  //全下げ

  edgeLine(L, 200, 170, 0, 90);
  tank(-13);
  startServoMoveByPin(1, -70, 255);  //閉じる
  waitServoTasks();
  servoDelay(130);
  startServoMoveByPin(0, -35, 255);  //上げる
  waitServoTasks();
  //フレーム前
  // tank(120, 120, 0);
  kabe(120);
  servoDelay(50);
  tank(-15);
  put_mosaic_both();
  startServoMoveByPin(0, -135, 255);  //全上げ
  waitServoTasks();
  pivot(33.0, 220, L);               //置く前と一緒だとずれる
  pivot(-33.0, 220, R);
  tank(-50);
  spin(180, 255);
  tline(120, 255, 0);
  tank(103);
  spin(90, 255);
  tline(320, 255, 0);
  tank(103);
  spin(90, 255);
  tline(1250, 200, 0);
  tank(93);
  spin(90, 255);
  servoDelay(1000);
  // ROI1
   int blueCount2 = 0;
    int greenCount2 = 0;
  int whiteCount2 = 0;
  int yellowCount2 = 0;
  int redCount2 = 0;

color = getRoiColor(1);

if (color == "B") {
  blueCount2++;
}
else if (color == "G") {
  greenCount2++;
}
else if (color == "W") {
  whiteCount2++;
}
else if (color == "Y") {
  yellowCount2++;
}
else if (color == "R") {
  redCount2++;
}


// ROI2
color = getRoiColor(2);

if (color == "B") {
  blueCount2++;
}
else if (color == "G") {
  greenCount2++;
}
else if (color == "W") {
  whiteCount2++;
}
else if (color == "Y") {
  yellowCount2++;
}
else if (color == "R") {
  redCount2++;
}


// ROI5
color = getRoiColor(5);

if (color == "B") {
  blueCount2++;
}
else if (color == "G") {
  greenCount2++;
}
else if (color == "W") {
  whiteCount2++;
}
else if (color == "Y") {
  yellowCount2++;
}
else if (color == "R") {
  redCount2++;
}

// ROI6
color = getRoiColor(6);

if (color == "B") {
  blueCount2++;
}
else if (color == "G") {
  greenCount2++;
}
else if (color == "W") {
  whiteCount2++;
}
else if (color == "Y") {
  yellowCount2++;
}
else if (color == "R") {
  redCount2++;
}
  if (! redCount2 >= 3){
  move_mosaic1('B', 'B');
    startServoMoveByPin(1, 0, 200);    //開ける
  startServoMoveByPin(0, 30, 255);  //全下げ
  edgeLine(R, 330, 160, 0, 90);
  tank(-12);
  startServoMoveByPin(1, -70, 255);  //閉じる
  waitServoTasks();
  servoDelay(130);
  startServoMoveByPin(0, -35, 255);  //上げる
  waitServoTasks();
  //フレーム前
  // tank(120, 120, 0);
  kabe(120);
  servoDelay(50);
  tank(-16);


  put_mosaic_both();  //グループ１置く動き

  startServoMoveByPin(0, -135, 255);  //全上げ
  waitServoTasks();

  startServoMoveByPin(2, -80, 255);  //ここで青を中に収納するたくらみ
  pivot(-32.0, 245, R);              //置く前と一緒だとずれる
  pivot(32.0, 245, L);
  startServoMoveByPin(2, 25, 200);
  waitServoTasks();
  spin(180, 220);
  tline(100, 200, 0);
  tank(83);
  spin(90, 255);
  }else{
   tline(320, 200, 0);
   tank(73);
  }
  
  move_mosaic2_3('Y', 'Y');
    startServoMoveByPin(1, 0, 200);    //開ける
  startServoMoveByPin(0, 30, 255);  //全下げ
  tline(600, 245, 0);
  tank(-7);
  startServoMoveByPin(1, -60, 255);  //閉じる
  waitServoTasks();
  servoDelay(100);
  startServoMoveByPin(0, -35, 255);  //上げる
  waitServoTasks();
  tank(80);
  spin(-90, 255);
  tline(100, 120);
  pivot(-44.5, 200, L, 100);//置くところ
  servoDelay(100);//servoDelay(50);
  pivot(44.5, 200, R, 100);
  tank(8, 80);
  servoDelay(100);
  startServoMoveByPin(0, 0, 255);  //全下げ
  waitServoTasks();
  startServoMoveByPin(1, 80, 200);  //開ける
  waitServoTasks();
  tank(-50);
  startServoMoveByPin(1, -60, 200);  //閉じる
  waitServoTasks();
  startServoMoveByPin(0, 20, 255);
  waitServoTasks();
  startServoMoveByPin(2, -110, 255);  //バックアーム展開
  waitServoTasks();
  servoDelay(50);
  tank(-17);
  pivot(-44.1, 255, R, 150);//-41.6
  pivot(44.1, 255, L, 150);//41.6戻るとこ
  tank(-60, 120);
  startServoMoveByPin(2, 25, 200);  //バックアームで黄色回収
  waitServoTasks();
  startServoMoveByPin(0, 30, 255);  //全下げ
  waitServoTasks();
  startServoMoveByPin(1, 0, 255);  //開ける
  waitServoTasks();
  tline(100, 120, 1, 110);
  tank(-39.5);
  startServoMoveByPin(1, -60, 200);  //閉じる
  waitServoTasks();
  tline(30, 100,1,90);
  startServoMoveByPin(0, -35, 255);  //上げ
  waitServoTasks();
  servoDelay(60);
  pivot(46.7, 200, R, 100);
  servoDelay(50);
  pivot(-44.6, 200, L, 100);
  servoDelay(50);
  tank(9);
  servoDelay(50);
  startServoMoveByPin(0, 20, 255);  //全下げ
  waitServoTasks();
  startServoMoveByPin(1, 0, 200);  //開ける
  waitServoTasks();
  servoDelay(200);
  startServoMoveByPin(0, -75, 255);  //全上げ
  waitServoTasks();
  startServoMoveByPin(1, -60, 200);  //閉める
  pivot(46.7, 200, L, 100);
  servoDelay(50);
  pivot(-44.6, 200, R, 100);
  if (getRoiColor(9),getRoiColor(10) == "R"){
      tank(-100);
        pivot(-44.5, 200, L, 100);//置くところ
        servoDelay(100);//servoDelay(50);
  pivot(44.5, 200, R, 100);
  tank(8, 80);
  servoDelay(100);
  front(F_CATCH, 80, 3);
  front(F_CATCH, 40, 75);
  pivot(-44.1, 255, R, 150);//-41.6
  pivot(44.1, 255, L, 150);//41.6戻るとこ
  tank(-100, 120);

  
  }  else if (getRoiColor(11),getRoiColor(12) == "R") {
      tank(-100);
        pivot(46.7, 200, R, 100);
  servoDelay(50);
  pivot(-44.6, 200, L, 100);
  servoDelay(50);
  tank(9);
  servoDelay(50);
  front(F_CATCH, 80,3);
  front(F_CATCH, 40, 75);
  pivot(46.7, 200, L, 100);
  servoDelay(50);
  pivot(-44.6, 200,R, 100);
  tank(-100);
  }else{
    tank(-100);
  }
  if (getRoiColor(1),getRoiColor(2),getRoiColor(5), getRoiColor(6) == "R"){
    spin(-90, 255);
    tline(576, 200, 0);
    tank(73);
    spin(90, 255);
    tline(320, 255);

  }
}

void extra_2() {

}

void extra_3() {
 
}

void extra_4() {

}

void extra_5() {

}



void extra_6() {

}  


void extra_7() {

}  



void extra_8() {

}  



void extra_9() {
}



void extra_10() {

}  



