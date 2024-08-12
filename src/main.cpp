#include <Arduino.h>
#include <Motor.h>
#include <PestoLink-Receive.h>
#include <Drivetrain.h>

DCMotor rawtop1 = DCMotor();
DCMotor rawbottom1 = DCMotor();
DCMotor rawtop2 = DCMotor();
DCMotor rawbottom2 = DCMotor();

DCDriver2PWM top1Driver = DCDriver2PWM(MOTOR1_A, MOTOR1_B);
DCDriver2PWM bottom1Driver = DCDriver2PWM(MOTOR4_A, MOTOR4_B);
DCDriver2PWM top2Driver = DCDriver2PWM(MOTOR2_A, MOTOR2_B);
DCDriver2PWM bottom2Driver = DCDriver2PWM(MOTOR3_A, MOTOR3_B);

Encoder top1Encoder = Encoder(4, 5, 1050);
Encoder bottom1Encoder = Encoder(2, 35, 1050);
Encoder top2Encoder = Encoder(TX, RX, 1050);
Encoder bottom2Encoder = Encoder(36, 39, 1050);

std::vector<double> top1Constants = {10.0, 5.0, 0};
std::vector<double> bottom1Constants = {10.0, 5.0, 0};
std::vector<double> top2Constants = {10.0, 5.0, 0};
std::vector<double> bottom2Constants = {10.0, 5.0, 0};

Motor top1 = Motor(4, 5, top1Constants, &rawtop1, &top1Driver, &top1Encoder);
Motor bottom1 = Motor(2, 35, bottom1Constants, &rawbottom1, &bottom1Driver, &bottom1Encoder);
Motor top2 = Motor(TX, RX, top2Constants, &rawtop2, &top2Driver, &top2Encoder);
Motor bottom2 = Motor(36, 39, bottom2Constants, &rawbottom2, &bottom2Driver, &bottom2Encoder);

Module right(&top1, &bottom1, RIGHT);
Module left(&top2, &bottom2, LEFT);

Drivetrain drivetrain(&left, &right);

double start = 0;

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

double applyDeadband(double value, double deadband) {
  if (abs(value) < deadband) {
    return 0;
  }
  return value;
}

void setup() {
  Serial.begin(115200);
  while(!Serial);
  char *localName = "MiniFRCDiffySwerve";
  PestoLink.begin(localName);

  SimpleFOCDebug::enable();

  top1Encoder.enableInterrupts(top1A, top1B);
  bottom1Encoder.enableInterrupts(bottom1A, bottom1B);
  // top2Encoder.enableInterrupts(top2A, top2B);
  // bottom2Encoder.enableInterrupts(bottom2A, bottom2B);

  top1Encoder.init();
  bottom1Encoder.init();
  // top2Encoder.init();
  // bottom2Encoder.init();

  drivetrain.begin();
}

void loop() {
  drivetrain.loop();

  double vxf = 0;
  double vyf = 0;
  double omega = 0;

  if (PestoLink.update()) {
    vxf = -applyDeadband(PestoLink.getAxis(0), 0.1);
    vyf = applyDeadband(PestoLink.getAxis(1), 0.1);
    omega = applyDeadband(PestoLink.getAxis(2), 0.1);

    if (vxf != 0 || vyf != 0 || omega != 0) {
      drivetrain.drive(vxf, vyf, omega);
    }
    else {
      drivetrain.stop();
    }
  }
}