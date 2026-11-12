#include "robot.h"
#include "motor.h"
#include "drive_sync.h"
#include <math.h>
#include <algorithm>

namespace {
float clip(float x, float lo, float hi) { return std::max(lo, std::min(x, hi)); }
float smooth(float x) { x=clip(x,0,1); return x*x*(3-2*x); }
// Integral of smoothstep, normalized ramp time.
float area(float x) { x=clip(x,0,1); return x*x*x-0.5f*x*x*x*x; }
float travel(float t, float ramp, float duration) {
  if(t<ramp) return ramp*area(t/ramp);
  if(t<duration-ramp) return t-0.5f*ramp;
  return duration-ramp-ramp*area((duration-t)/ramp);
}
int32_t difference(int32_t a,int32_t b) {
  return static_cast<int32_t>(static_cast<uint32_t>(a)-static_cast<uint32_t>(b));
}
}

bool Robot::drive(float lmax,float rmax,float lmin,float rmin,
                  float leftMm,float rightMm,unsigned long timeoutMs) {
  stop();
  const float values[]={lmax,rmax,lmin,rmin,leftMm,rightMm,
    config_.leftWheelDiameterMm,config_.rightWheelDiameterMm,
    config_.leftCountsPerRev,config_.rightCountsPerRev,config_.speedKp,
    config_.speedKi,config_.positionKp,config_.syncKp,
    config_.leftPwmPerMmS,config_.rightPwmPerMmS,
    config_.positionToleranceMm,config_.rampTimeSeconds};
  for(float v:values) if(!isfinite(v)) return false;
  if(!initialized_ || !timeoutMs || config_.controlPeriodMs<1 ||
     config_.controlPeriodMs>100 || !config_.settleTimeMs ||
     config_.brakeSampleMs<config_.controlPeriodMs || config_.brakeStopCounts<0 ||
     config_.rampTimeSeconds<=0 || config_.positionToleranceMm<=0 ||
     config_.leftWheelDiameterMm<=0 || config_.rightWheelDiameterMm<=0 ||
     config_.leftCountsPerRev<=0 || config_.rightCountsPerRev<=0 ||
     config_.speedKp<0 || config_.speedKi<0 || config_.positionKp<=0 ||
     config_.syncKp<0 || config_.leftPwmPerMmS<=0 || config_.rightPwmPerMmS<=0 ||
     lmin<0 || rmin<0 || lmax<lmin || rmax<rmin ||
     (leftMm!=0 && lmax<=0) || (rightMm!=0 && rmax<=0)) return false;
  const float target[2]={fabsf(leftMm),fabsf(rightMm)};
  if(target[0]==0 && target[1]==0) return true;
  const float direction[2]={leftMm<0?-1.0f:1.0f,rightMm<0?-1.0f:1.0f};
  const float scale[2]={config_.leftWheelDiameterMm*3.14159265359f/config_.leftCountsPerRev,
                        config_.rightWheelDiameterMm*3.14159265359f/config_.rightCountsPerRev};
  for(int i=0;i<2;++i) if(!isfinite(scale[i]) || scale[i]<=0 || target[i]/scale[i]>2147480000.0f) return false;
  float cruiseTime=std::max(target[0]?target[0]/lmax:0.0f,target[1]?target[1]/rmax:0.0f);
  // MIN constrains compatible cruising ratios; ramps start/end at zero.
  if((target[0] && target[0]/cruiseTime<lmin) ||
     (target[1] && target[1]/cruiseTime<rmin)) return false;
  float ramp=std::min(config_.rampTimeSeconds,cruiseTime);
  float duration=cruiseTime+ramp;
  if(!isfinite(duration)) return false;
  float peak[2]={target[0]/cruiseTime,target[1]/cruiseTime};
  float integral[2]={0,0},previous[2]={0,0};
  const float feed[2]={config_.leftPwmPerMmS,config_.rightPwmPerMmS};
  const EncoderCounts start=counts();
  const unsigned long began=millis();
  unsigned long last=began,stableSince=began;
  unsigned long brakeWindowStart[2]={began,began};
  uint64_t brakeMovement[2]={0,0};
  bool brakeLowMotion[2]={false,false};
  int motionDirection[2]={0,0};
  int32_t previousCount[2]={start.left,start.right};
  bool settling=false;
  bool correctionStalled=false;
  unsigned long correctionStalledSince=began;
  while(millis()-began<timeoutMs) {
    encoderPrintSerial();
    unsigned long now=millis();
    if(now-last<config_.controlPeriodMs) { delay(1); continue; }
    float dt=(now-last)*0.001f; last=now;
    float t=std::min((now-began)*0.001f,duration);
    EncoderCounts c=counts();
    float position[2]={difference(c.left,start.left)*scale[0]*direction[0],
                       difference(c.right,start.right)*scale[1]*direction[1]};
    float fraction=travel(t,ramp,duration)/cruiseTime;
    float envelope=smooth(t/ramp)*smooth((duration-t)/ramp);
    float baseSpeed[2];
    for(int i=0;i<2;++i) {
      baseSpeed[i]=clip(peak[i]*envelope+
        config_.positionKp*(target[i]*fraction-position[i]),-peak[i],peak[i]);
    }
    const auto synced=drive_sync::apply(baseSpeed[0],baseSpeed[1],peak[0],peak[1],
      target[0],target[1],position[0],position[1],config_.syncKp,t<duration);
    const float desiredSpeed[2]={synced.left,synced.right};
    bool arrived=t>=duration;
    bool blockedCorrection=false;
    bool allLowMotion=true;
    int output[2]={0,0};
    for(int i=0;i<2;++i) {
      const int32_t rawCount=i==0?c.left:c.right;
      const int32_t delta=difference(rawCount,previousCount[i]);
      previousCount[i]=rawCount;
      if(delta!=0) {
        motionDirection[i]=(delta>0?1:-1)*static_cast<int>(direction[i]);
      }
      // サンプル間の差の絶対値を合計し、往復による相殺を避ける。
      const int64_t movement=delta;
      brakeMovement[i]+=static_cast<uint64_t>(movement<0?-movement:movement);
      if(brakeMovement[i]>static_cast<uint64_t>(config_.brakeStopCounts))
        brakeLowMotion[i]=false;
      if(now-brakeWindowStart[i]>=config_.brakeSampleMs) {
        brakeLowMotion[i]=brakeMovement[i]<=static_cast<uint64_t>(config_.brakeStopCounts);
        brakeWindowStart[i]=now;
        brakeMovement[i]=0;
      }
      float speed=(position[i]-previous[i])/dt; previous[i]=position[i];
      if(!target[i]) continue;
      allLowMotion=allLowMotion && brakeLowMotion[i];
      float error=target[i]-position[i];
      bool atGoal=fabsf(error)<=config_.positionToleranceMm;
      arrived=arrived && atGoal && fabsf(speed)<=scale[i]/dt;
      const float desired=desiredSpeed[i];
      if(t>=duration && atGoal) { integral[i]=0; continue; }
      float speedError=desired-speed;
      float next=clip(integral[i]+config_.speedKi*speedError*dt,-255,255);
      float pwm=feed[i]*desired+config_.speedKp*speedError+next;
      if(fabsf(pwm)<=255 || pwm*speedError<0) integral[i]=next;
      // 走行方向に正規化したpwm<0は、前進・後退とも指令と逆の出力。
      // 停止判定後は位置補正由来の逆出力も切り、積分で復活させない。
      const bool reverseBraking=motionDirection[i]!=0 &&
        pwm*motionDirection[i]<0 && desired*motionDirection[i]>=0;
      if((reverseBraking || pwm<0) && brakeLowMotion[i]) {
        blockedCorrection=blockedCorrection ||
          (t>=duration && !atGoal && error<0);
        integral[i]=0;
        pwm=0;
      }
      output[i]=static_cast<int>(lroundf(clip(pwm,-255,255)*direction[i]));
    }
    motorPower(output[0],output[1]);
    // 目標を越えて停止し、逆補正を切ったままなら成功扱いせず終了する。
    // 両輪の出力OFF・低速が継続した場合だけ判定する。
    if(blockedCorrection && allLowMotion && output[0]==0 && output[1]==0) {
      if(!correctionStalled) { correctionStalledSince=now; correctionStalled=true; }
      if(now-correctionStalledSince>=config_.settleTimeMs) { stop(); return false; }
    } else correctionStalled=false;
    if(arrived) {
      if(!settling) { stableSince=now; settling=true; }
      if(now-stableSince>=config_.settleTimeMs) { stop(); return true; }
    } else settling=false;
  }
  stop(); return false;
}
