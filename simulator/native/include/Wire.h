#pragma once
#include "Arduino.h"
#include <vector>
class TwoWire {
 public:
  void setSDA(int){} void setSCL(int){} void begin(){} void begin(int,int){} void setClock(uint32_t){}
  void beginTransmission(uint8_t a){addr=a;tx.clear();} size_t write(uint8_t b){tx.push_back(b);return 1;}
  uint8_t endTransmission(bool stop=true); int requestFrom(int address,int count,bool stop=true);
  int available()const{return (int)(rx.size()-pos);} int read(){return available()?rx[pos++]:-1;}
 private:uint8_t addr=0,lastReg=0;std::vector<uint8_t>tx,rx;size_t pos=0;
};
extern TwoWire Wire,Wire1;
