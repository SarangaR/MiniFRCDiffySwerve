#include <Arduino.h>
#include <Motor.h>
#include <PestoLink-Receive.h>
#include <Drivetrain.h>
#include "Filters/Butterworth.hpp"
#include <Alfredo_NoU3.h>
#include <ESP32Encoder.h>

#define DEBUG_MODE_SERIAL false

NoU_Motor rawtop1(3);
NoU_Motor rawbottom1(4);
NoU_Motor rawtop2(5);
NoU_Motor rawbottom2(2);
NoU_Motor rawtop3(7);
NoU_Motor rawbottom3(6);

float reduction_ratio = 50;
float ppr = 7;
float cpr = ppr * reduction_ratio;


Encoder top1Encoder = Encoder(16, 15, cpr);
Encoder bottom1Encoder = Encoder(11, 10, cpr);
Encoder top2Encoder = Encoder(41, 42, cpr);
Encoder bottom2Encoder = Encoder(18, 17, cpr);
Encoder top3Encoder = Encoder(38, 37, cpr);
Encoder bottom3Encoder = Encoder(40, 39, cpr);

int encoderPins[] = {16, 15, 11, 10, 41, 42, 18, 17, 38, 37, 40, 39};

void top1A() {
  top1Encoder.handleA();
}
void top1B() {
  top1Encoder.handleB();
}
void bottom1A() {
  bottom1Encoder.handleA();
}
void bottom1B() {
  bottom1Encoder.handleB();
}
void top2A() {
  top2Encoder.handleA();
}
void top2B() {
  top2Encoder.handleB();
} 
void bottom2A() {
  bottom2Encoder.handleA();
}
void bottom2B() {
  bottom2Encoder.handleB();
}
void top3A() {
  top3Encoder.handleA();
}
void top3B() {
  top3Encoder.handleB();
}
void bottom3A() {
  bottom3Encoder.handleA();
}
void bottom3B() {
  bottom3Encoder.handleB();
}

std::array<float, 3> top1Constants = {5.0f, 5.0f, 0};
std::array<float, 3> bottom1Constants = {5.0f, 5.0f, 0};
std::array<float, 3> top2Constants = {5.0f, 5.0f, 0};
std::array<float, 3> bottom2Constants = {5.0f, 5.0f, 0};
std::array<float, 3> top3Constants = {5.0f, 5.0f, 0};
std::array<float, 3> bottom3Constants = {5.0f, 5.0f, 0};

Motor top1 = Motor(4, 5, top1Constants, &rawtop1, &top1Encoder);
Motor bottom1 = Motor(2, 35, bottom1Constants, &rawbottom1, &bottom1Encoder);
Motor top2 = Motor(36, 16, top2Constants, &rawtop2, &top2Encoder);
Motor bottom2 = Motor(39, 49, bottom2Constants, &rawbottom2, &bottom2Encoder);
Motor top3 = Motor(14, 34, top3Constants, &rawtop3, &top3Encoder);
Motor bottom3 = Motor(13, 12, bottom3Constants, &rawbottom3, &bottom3Encoder);

Module right(&top1, &bottom1, RIGHT);
Module left(&top2, &bottom2, LEFT);
Module center(&top3, &bottom3, CENTER);

Drivetrain drivetrain(&left, &right, &center);

const float MAX_SPEED = Module::MAX_SPEED_SPIN_MS;
const float CIRCUMFERENCE = 0.0254f * 5.0f * M_PI;
const float ONE_ROTATION_S = CIRCUMFERENCE / MAX_SPEED;
const float MAX_ROT = (2*M_PI) / ONE_ROTATION_S;

float applyDeadband(float value, float deadband) {
  if (abs(value) < deadband) {
    return 0;
  }
  return value;
}

HWCDC *SerialPtr;

void setup() {
  NoU3.begin();
  Serial.begin(115200);
  #if DEBUG_MODE_SERIAL
    while(!Serial);
  #endif

  char *localName = "MiniFRCDiffySwerve";
  PestoLink.begin(localName);

  for (int i = 0; i < 12; i++) {
    pinMode(encoderPins[i], INPUT);
  }

  top1Encoder.enableInterrupts(top1A, top1B);
  bottom1Encoder.enableInterrupts(bottom1A, bottom1B);
  top2Encoder.enableInterrupts(top2A, top2B);
  bottom2Encoder.enableInterrupts(bottom2A, bottom2B);
  top3Encoder.enableInterrupts(top3A, top3B);
  bottom3Encoder.enableInterrupts(bottom3A, bottom3B);

  top1Encoder.init();
  bottom1Encoder.init();
  top2Encoder.init();
  bottom2Encoder.init();
  top3Encoder.init();
  bottom3Encoder.init();
  
  RSL::initialize();

  SerialPtr = &Serial;
}

Angle gyro = Angle(0, DEGREES);
// Eigen::MatrixXd prev_matrix = Eigen::MatrixXd(3, 1);

void loop() {
  float vxf = 0;
  float vyf = 0;
  float omega = 0;
  if (PestoLink.update()) {
    RSL::setState(RSL_ENABLED);
    PestoLink.setBatteryVal(NoU3.getBatteryVoltage());
    vxf = -applyDeadband(PestoLink.getAxis(1), 0.1);//*MAX_SPEED;
    vyf = applyDeadband(PestoLink.getAxis(0), 0.1);//*MAX_SPEED;
    omega = applyDeadband(PestoLink.getAxis(2), 0.1);//*MAX_ROT;

    if (vxf != 0 || vyf != 0 || omega != 0) {
      auto statesPair = drivetrain.drive(vxf, vyf, omega, gyro, false, SerialPtr);
      // Serial.println("Left: " + states[0].toString() + " Right: " + states[1].toString() + " Center: " + states[2].toString());
    }
    else {
      drivetrain.stop();
    }
  }
  else {
    RSL::setState(RSL_ON);
  }

  // drivetrain.loop();
  // RSL::update();
  // top2.setVelocity(Angle(10*PI));
  // bottom2.setVelocity(Angle(10*PI));
  // rawbottom2.set(-1);
}