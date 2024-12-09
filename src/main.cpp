#include <Arduino.h>
#include <Motor.h>
#include <PestoLink-Receive.h>
#include <Drivetrain.h>
#include "Filters/Butterworth.hpp"
#include <Alfredo_NoU3.h>
#include <ESP32Encoder.h>
#include <SparkFun_Qwiic_OTOS_Arduino_Library.h>
#include <RobotPose.h>

#define DEBUG_MODE_SERIAL false
#define MAG_DEC 14.44504

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

Motor top1 = Motor(16, 15, &rawtop1, &top1Encoder);
Motor bottom1 = Motor(11, 10, &rawbottom1, &bottom1Encoder);
Motor top2 = Motor(41, 42, &rawtop2, &top2Encoder);
Motor bottom2 = Motor(18, 17, &rawbottom2, &bottom2Encoder);
Motor top3 = Motor(38, 37, &rawtop3, &top3Encoder);
Motor bottom3 = Motor(40, 39, &rawbottom3, &bottom3Encoder);

Module right(&top1, &bottom1, RIGHT);
Module left(&top2, &bottom2, LEFT);
Module center(&top3, &bottom3, CENTER);

Drivetrain drivetrain(&left, &right, &center);

const float MAX_SPEED = Module::MAX_SPEED_SPIN_MS;

float applyDeadband(float value, float deadband) {
  if (abs(value) < deadband) {
    return 0;
  }
  return value;
}

float wrapYaw(float currentYaw) {
  static bool isInitialized = false;
  static int rotations = 0;
  static float previousYaw = 0;


  if(isInitialized == false){
    previousYaw = currentYaw;
    isInitialized = true;
  }

    // Check for wrapping
    if (currentYaw - previousYaw > 180.0) {
        rotations--; // Wrapped from -180 to 180
    } else if (currentYaw - previousYaw < -180.0) {
        rotations++; // Wrapped from 180 to -180
    }

    // Wrap the yaw angle between -180 and 180
    float wrappedYaw = currentYaw + rotations * 360.0;

    previousYaw = currentYaw;

    return wrappedYaw;
}

HWCDC *SerialPtr;

RobotPose robotPose = RobotPose();

float g = 0;

void setup() {
  NoU3.begin();
  Serial.begin(115200);
  #if DEBUG_MODE_SERIAL
    while(!Serial);
  #endif

  PestoLink.begin("BumBotics");

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

  left.setMotorInvert(false, true);
  right.setMotorInvert(true, true);

  SerialPtr = &Serial;

  while (!robotPose.begin(MAG_DEC)) {
    Serial.println("OTOS not connected, check your wiring and I2C address!");
    delay(1000);
  }

  NoU3.updateIMUs();
  robotPose.setYawOffset(NoU3.magnetometer_x, NoU3.magnetometer_y);
  // robotPose.setOTOSAngularScalar(1.0);
}

float yaw_gyro_deg = 0;
float yaw_mag_deg = 0;
float wrapped_yaw_mag_deg = 0;
float yaw = 0;

float yaw_offset = 0;

const float alpha = 0.98; // Complementary filter weighting
unsigned long lastTime = 0;

float gyro_z_offset_degrees = -0.785;

void loop() {
  float vxf = 0;
  float vyf = 0;
  float omega = 0;
  if (PestoLink.update()) {
    if (PestoLink.buttonHeld(0)) {
      PestoLink.print(String(yaw).c_str());  
    }
    else {
      PestoLink.printBatteryVoltage(NoU3.getBatteryVoltage());
    }
    vxf = applyDeadband(PestoLink.getAxis(1), 0.1);
    vyf = -applyDeadband(PestoLink.getAxis(0), 0.1);
    omega = -applyDeadband(PestoLink.getAxis(2), 0.1)*4*M_PI;

    if (PestoLink.buttonHeld(8)) {
      yaw_offset = yaw;
    }


    if (vxf != 0 || vyf != 0 || omega != 0) {
      drivetrain.drive(vxf, vyf, omega, Angle(yaw, DEGREES), true, SerialPtr);
    }
    else {
      drivetrain.stop();
    }
  }

  drivetrain.loop();
  if (NoU3.updateIMUs()) {
    unsigned long currentTime = millis();
    float dt = (currentTime - lastTime) / 1000.0;
    lastTime = currentTime;

    // Gyroscope yaw update
    yaw_gyro_deg += (NoU3.gyroscope_z - gyro_z_offset_degrees) * dt;

    // Magnetometer yaw
    yaw_mag_deg = -1 * degrees(atan2(NoU3.magnetometer_y, NoU3.magnetometer_x));

    // Wrap Magnetometer yaw
    if (yaw_mag_deg > 180.0) {
      yaw_mag_deg -= 360.0;
    } else if (yaw_mag_deg < -180.0) {
      yaw_mag_deg += 360.0;
    }
    wrapped_yaw_mag_deg = yaw_mag_deg;

    // Apply complementary filter with drift compensation
    yaw = alpha * (yaw_gyro_deg) + (1 - alpha) * wrapped_yaw_mag_deg - yaw_offset;

    // yaw = wrapYaw(yaw);

    if (yaw > 180.0) {
      yaw -= 360.0;
    } else if (yaw < -180.0) {
      yaw += 360.0;
    }

  }
  // Serial.println("X: " + String(robotPose.getPosition().x) + " Y: " + String(robotPose.getPosition().y) + " H: " + String(robotPose.getHeading().wrapNeg180To180().getDegrees()));
}