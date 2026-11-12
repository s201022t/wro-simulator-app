#ifdef DRIVE_FF_CALIBRATION
#include "ff_calibration.h"
#include "robot.h"
#include "ff_motion_detector.h"
#include <stdlib.h>

namespace {
// PWM values, not mm/s. Each attempt starts from output OFF.
constexpr int PWM_STEP=5;
constexpr int MAX_TEST_PWM=150;
constexpr unsigned long TEST_MS=600;
constexpr unsigned long REST_MS=700;
constexpr int REST_JITTER_COUNTS=2;
using namespace ff_calibration;

int32_t delta(int32_t now,int32_t start) {
  return static_cast<int32_t>(static_cast<uint32_t>(now)-static_cast<uint32_t>(start));
}
bool pauseStopped(Robot& robot,int button,unsigned long duration) {
  robot.stop();
  const unsigned long start=millis();
  while(millis()-start<duration) {
    if(digitalRead(button)==LOW) return false;
    delay(1);
  }
  return true;
}

// Returns a measured threshold, or -1 for cancellation/wiring error/no motion.
int measure(Robot& robot,int button,bool left,int direction) {
  Serial.print(left ? "LEFT " : "RIGHT ");
  Serial.println(direction>0 ? "FORWARD" : "REVERSE");
  for(int pwm=PWM_STEP;pwm<=MAX_TEST_PWM;pwm+=PWM_STEP) {
    if(!pauseStopped(robot,button,REST_MS)) return -1;
    const EncoderCounts start=robot.counts();
    // Allow tiny jitter, but reject movement across a wider range even if it returns.
    int32_t low[2]={0,0},high[2]={0,0};
    const unsigned long restStart=millis();
    while(millis()-restStart<100) {
      if(digitalRead(button)==LOW) return -1;
      const EncoderCounts now=robot.counts();
      const int32_t offset[2]={delta(now.left,start.left),delta(now.right,start.right)};
      for(int i=0;i<2;++i) {
        if(offset[i]<low[i]) low[i]=offset[i];
        if(offset[i]>high[i]) high[i]=offset[i];
      }
      delay(1);
    }
    const EncoderCounts before=robot.counts();
    if(static_cast<int64_t>(high[0])-low[0]>REST_JITTER_COUNTS ||
       static_cast<int64_t>(high[1])-low[1]>REST_JITTER_COUNTS) {
      Serial.println("ERROR: wheels still moving before test. Keep wheels still / increase REST_MS.");
      return -1;
    }
    Serial.print("TEST PWM="); Serial.println(pwm);
    const unsigned long began=millis();
    robot.setPower(left ? direction*pwm : 0,left ? 0 : direction*pwm);
    int32_t dl=0,dr=0;
    bool detected=false;
    MotionDetector motion;
    unsigned long windowStart=began;
    const char* fault=nullptr;
    while(millis()-began<TEST_MS) {
      if(digitalRead(button)==LOW) { robot.stop(); return -1; }
      const EncoderCounts now=robot.counts();
      dl=delta(now.left,before.left); dr=delta(now.right,before.right);
      const int64_t progress=static_cast<int64_t>(left ? dl : dr)*direction;
      const int32_t other=left ? dr : dl;
      // Fault checks remain immediate; only success requires sustained movement.
      if(llabs(static_cast<long long>(other))>=MOTION_COUNTS) {
        fault="ERROR: other encoder moved. Check encoder L/R mapping or unpowered wheel motion.";
        break;
      }
      if(progress<=-MOTION_COUNTS) {
        fault="ERROR: encoder sign differs from commanded direction.";
        break;
      }
      if(millis()-windowStart>=WINDOW_MS) {
        windowStart=millis();
        if(motion.sample(progress)) { detected=true; break; }
      }
      delay(1);
    }
    robot.stop();
    Serial.print("  dL="); Serial.print(dl);
    Serial.print(" dR="); Serial.println(dr);
    if(fault) { Serial.println(fault); return -1; }
    if(detected) {
      Serial.print("BREAKAWAY PWM="); Serial.println(pwm);
      return pwm;
    }
  }
  Serial.println("ERROR: no motion detected up to MAX_TEST_PWM. Check wiring/load; no threshold recorded.");
  return -1;
}
}

bool runFfCalibration(Robot& robot,int stopButtonPin) {
  robot.stop();
  Serial.println("FF CALIBRATION v2 (sustained motion): release button to begin; press to abort.");
  while(digitalRead(stopButtonPin)==LOW) delay(1);
  if(!pauseStopped(robot,stopButtonPin,REST_MS)) return false;
  int result[4];
  for(int i=0;i<4;++i) {
    result[i]=measure(robot,stopButtonPin,i<2,(i%2==0)?1:-1);
    if(result[i]<0) {
      robot.stop();
      Serial.println("CALIBRATION ABORTED: no complete result. Press/release to retry.");
      return false;
    }
  }
  robot.stop();
  Serial.println("=== BREAKAWAY RESULTS (PWM / 255) ===");
  const char* labels[]={"LEFT_FORWARD=","LEFT_REVERSE=","RIGHT_FORWARD=","RIGHT_REVERSE="};
  for(int i=0;i<4;++i) { Serial.print(labels[i]); Serial.println(result[i]); }
  Serial.println("Resolution: 5 PWM; observation: 600 ms; total >=12 counts; >=2 counts/50ms for 3 consecutive windows.");
  Serial.println("Repeat 3 times. Values are breakaway estimates, NOT automatically saved FF gains.");
  return true;
}
#endif
