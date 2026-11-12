#include "Arduino.h"
#include "Wire.h"
#include <array>
#include <atomic>
#include <cmath>
#include <mutex>

HardwareSerial Serial,Serial1; TwoWire Wire,Wire1;
extern int servoPins[4];
extern int servoPulseUs[4];
extern volatile long enc1,enc2;
namespace {
struct World { uint64_t us=0,lastEmit=0; double x=240,y=900,heading=-90,lv=0,rv=0,yawRate=0,lphase=0,rphase=0; int la=0,lb=0,ra=0,rb=0,pwmL=0,pwmR=0; std::array<int,64> digital{}; std::array<int,64> analog{}; std::array<void(*)(),64> isr{}; bool silent=false; } w;
constexpr double wheelD=62.4, track=167.0, ppr=620.0, maxMmS=900.0;
void encoder(bool left,int dir){int a=dir>0?1:0,b=left?(dir>0?1:0):(dir>0?0:1);int pa=left?26:28,pb=left?27:29;w.digital[pa]=a;w.digital[pb]=b;if(w.isr[pa])w.isr[pa]();}
int lineAt(double sx,double sy){bool black=fmod(std::abs(sx),400.0)<18||fmod(std::abs(sy),300.0)<18;return black?930:110;}
}
long map(long x,long a,long b,long c,long d){return (x-a)*(d-c)/(b-a)+c;}
void HardwareSerial::emit(const std::string&s){if(this==&Serial1){tx_+=s;if(tx_.find('\n')!=std::string::npos)cameraReply();return;}if(!w.silent)std::cerr<<s;}
void HardwareSerial::cameraReply(){std::string command=tx_.substr(0,tx_.find('\n'));tx_.clear();if(command!="READ"&&command!="READ_ROI")return;auto colorAt=[](double x,double y){if(y<360){if(x<590)return std::string("Y");if(x<1181)return std::string("B");if(x<1771)return std::string("G");return std::string("W");}if(x>900&&x<1460&&y>430&&y<710)return std::string("W");return std::string("UNKNOWN");};for(int i=1;i<=12;i++){int row=(i-1)/4,col=(i-1)%4;double rad=w.heading*PI/180.0,forward=180+row*110,lateral=(col-1.5)*90;std::string c=colorAt(w.x+cos(rad)*forward-sin(rad)*lateral,w.y+sin(rad)*forward+cos(rad)*lateral);std::string roi="ROI,"+std::to_string(i)+","+c+"\n";int r=c=="G"?35:c=="B"?30:c=="W"?220:c=="Y"?230:70,g=c=="G"?180:c=="B"?70:c=="W"?220:c=="Y"?190:70,b=c=="B"?200:c=="W"?220:35;std::string rgb="RGB,"+std::to_string(i)+","+std::to_string(r)+","+std::to_string(g)+","+std::to_string(b)+"\n";for(char x:roi)rx_.push_back(x);for(char x:rgb)rx_.push_back(x);}for(char x:std::string("END\n"))rx_.push_back(x);}
void pinMode(int,int){} int digitalPinToInterrupt(int p){return p;} void attachInterrupt(int p,void(*f)(),int){if(p>=0&&p<64)w.isr[p]=f;}
int digitalRead(int p){if(p==20)return HIGH;return p>=0&&p<64?w.digital[p]:LOW;} void digitalWrite(int p,int v){if(p>=0&&p<64)w.digital[p]=v;}
void analogWrite(int p,int v){v=constrain(v,0,255);w.analog[p]=v;if(p==10||p==11){w.la=w.analog[10];w.lb=w.analog[11];w.pwmL=w.la-w.lb;}if(p==8||p==9){w.ra=w.analog[8];w.rb=w.analog[9];w.pwmR=w.ra-w.rb;}}
int analogRead(int p){double rad=w.heading*PI/180.0;double forward=75,lateral=0;if(p==26)lateral=70;if(p==27)lateral=25;if(p==28)lateral=-25;if(p==29)lateral=-70;double sx=w.x+cos(rad)*forward-sin(rad)*lateral,sy=w.y+sin(rad)*forward+cos(rad)*lateral;return lineAt(sx,sy);}
unsigned long micros(){sim::advance(125);return (unsigned long)w.us;} unsigned long millis(){return (unsigned long)(w.us/1000);} void delay(unsigned long m){sim::advance((uint64_t)m*1000);} void delayMicroseconds(unsigned int u){sim::advance(u);} void yield(){sim::advance(125);} void noInterrupts(){}void interrupts(){}void tone(int,unsigned int){}void noTone(int){}
namespace sim {
void reset(){w=World{};w.x=240;w.y=900;w.heading=-90;} void setProgram(const std::string&){} bool quiet(){return w.silent;}
void advance(uint64_t us){const double dt=us/1e6;w.us+=us;double targetL=w.pwmL/255.0*maxMmS,targetR=w.pwmR/255.0*maxMmS;double blend=1-std::exp(-dt/0.08);w.lv+=(targetL-w.lv)*blend;w.rv+=(targetR-w.rv)*blend;double dl=w.lv*dt,dr=w.rv*dt,dtheta=(dr-dl)/track;w.yawRate=(w.rv-w.lv)/track*180/PI;double h=w.heading*PI/180.0+dtheta/2;w.x+=(dl+dr)/2*cos(h);w.y+=(dl+dr)/2*sin(h);w.heading+=dtheta*180/PI;w.lphase+=std::abs(dl)/(PI*wheelD)*ppr;w.rphase+=std::abs(dr)/(PI*wheelD)*ppr;while(w.lphase>=1){encoder(true,w.lv>=0?1:-1);w.lphase-=1;}while(w.rphase>=1){encoder(false,w.rv>=0?1:-1);w.rphase-=1;}emitState();}
void emitState(bool force){if(!force&&w.us-w.lastEmit<20000)return;w.lastEmit=w.us;std::cout<<"{\"type\":\"state\",\"time\":"<<w.us/1e6<<",\"x\":"<<w.x<<",\"y\":"<<w.y<<",\"heading\":"<<w.heading<<",\"yawRate\":"<<w.yawRate<<",\"pwmLeft\":"<<w.pwmL<<",\"pwmRight\":"<<w.pwmR<<",\"velocityLeft\":"<<w.lv<<",\"velocityRight\":"<<w.rv<<",\"encoder\":["<<enc2<<","<<enc1<<"],\"line\":["<<analogRead(26)<<","<<analogRead(27)<<","<<analogRead(28)<<","<<analogRead(29)<<"],\"servo\":["<<servoPulseUs[0]<<","<<servoPulseUs[1]<<","<<servoPulseUs[2]<<","<<servoPulseUs[3]<<"]}"<<std::endl;}
}
uint8_t TwoWire::endTransmission(bool){if(!tx.empty())lastReg=tx[0];return addr==0x68||addr==0x69?0:0;} int TwoWire::requestFrom(int,int count,bool){rx.clear();pos=0;if(lastReg==0x75&&count==1)rx.push_back(0x71);else{int16_t z=(int16_t)std::lround(-w.yawRate*32.8);rx.push_back((z>>8)&255);rx.push_back(z&255);while((int)rx.size()<count)rx.push_back(0);}return count;}
