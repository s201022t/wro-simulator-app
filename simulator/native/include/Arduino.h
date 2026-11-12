#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <deque>
#include <sstream>
#include <string>

using byte = uint8_t;
using boolean = bool;
#define HIGH 1
#define LOW 0
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2
#define CHANGE 3
#define RISING 4
#define FALLING 5
#define HEX 16
#define PI 3.14159265358979323846
#define RAD_TO_DEG 57.29577951308232

template<class T, class U, class V> constexpr auto constrain(T x, U a, V b) { return std::clamp(x, static_cast<T>(a), static_cast<T>(b)); }
template<class T, class U> constexpr auto min(T a, U b) { return a < static_cast<T>(b) ? a : static_cast<T>(b); }
template<class T, class U> constexpr auto max(T a, U b) { return a > static_cast<T>(b) ? a : static_cast<T>(b); }
template<class T> constexpr T sq(T x){return x*x;}
long map(long x, long in_min, long in_max, long out_min, long out_max);

class String {
 public:
  String() = default; String(const char* s): v_(s?s:"") {} String(const std::string& s):v_(s){}
  String(char c):v_(1,c){} String(int n):v_(std::to_string(n)){} String(long n):v_(std::to_string(n)){}
  size_t length() const{return v_.size();} const char* c_str() const{return v_.c_str();}
  int indexOf(char c, int from=0) const { auto p=v_.find(c,from); return p==std::string::npos?-1:(int)p; }
  int indexOf(const char* s, int from=0) const { auto p=v_.find(s,from); return p==std::string::npos?-1:(int)p; }
  bool startsWith(const char* s) const { return s && v_.rfind(s,0)==0; }
  String substring(int a, int b=-1) const { if(a<0)a=0; return v_.substr(a,b<0?std::string::npos:b-a); }
  void trim(){auto a=v_.find_first_not_of(" \r\n\t"),b=v_.find_last_not_of(" \r\n\t");v_=a==std::string::npos?"":v_.substr(a,b-a+1);}
  void toUpperCase(){for(char&c:v_)c=static_cast<char>(std::toupper((unsigned char)c));}
  int toInt() const {try{return std::stoi(v_);}catch(...){return 0;}}
  float toFloat() const {try{return std::stof(v_);}catch(...){return 0;}}
  char charAt(int i) const{return i>=0&&i<(int)v_.size()?v_[i]:0;}
  String& operator+=(const String&o){v_+=o.v_;return *this;} String& operator+=(char c){v_+=c;return *this;}
  friend String operator+(String a,const String&b){return a+=b;}
  friend bool operator==(const String&a,const char*b){return a.v_==(b?b:"");} friend bool operator!=(const String&a,const char*b){return !(a==b);}
  friend bool operator==(const String&a,const String&b){return a.v_==b.v_;}
  operator std::string()const{return v_;}
 private: std::string v_;
};

class HardwareSerial {
 public:
  void begin(unsigned long, int=0, int=-1, int=-1){} void setTX(int){} void setRX(int){}
  operator bool() const{return true;} int available()const{return (int)rx_.size();} int read(){if(rx_.empty())return -1;int c=rx_.front();rx_.pop_front();return c;} void flush(){}
  void print(const String&s){emit(std::string(s));} void print(const char*s){emit(s?s:"");} void print(char c){emit(std::string(1,c));}
  template<class T> void print(T x){std::ostringstream o;o<<x;emit(o.str());}
  template<class T> void print(T x,int base){std::ostringstream o;if(base==16)o<<std::hex<<(uint64_t)x;else o<<x;emit(o.str());}
  void print(float x,int d){std::ostringstream o;o<<std::fixed<<std::setprecision(d)<<x;emit(o.str());}
  void print(double x,int d){print((float)x,d);} template<class T> void println(T x){print(x);println();}
  void println(float x,int d){print(x,d);println();} void println(){emit("\n");}
  template<class T> void println(T x,int base){print(x,base);println();}
  size_t write(uint8_t){return 1;} size_t write(const uint8_t*,size_t n){return n;}
 private: void emit(const std::string&s); std::string tx_; std::deque<uint8_t> rx_; void cameraReply();
};
extern HardwareSerial Serial, Serial1;

void pinMode(int,int); int digitalRead(int); void digitalWrite(int,int); int analogRead(int); void analogWrite(int,int);
void attachInterrupt(int,void(*)(),int); int digitalPinToInterrupt(int);
unsigned long micros(); unsigned long millis(); void delay(unsigned long); void delayMicroseconds(unsigned int); void yield();
void noInterrupts(); void interrupts(); void tone(int,unsigned int); void noTone(int);
namespace sim { void reset(); void advance(uint64_t us); void setProgram(const std::string&); void emitState(bool force=false); bool quiet(); }
