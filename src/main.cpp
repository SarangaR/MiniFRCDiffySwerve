#include <Arduino.h>
#include <Motor.h>
#include <PestoLink-Receive.h>
#include <Drivetrain.h>

NoU_Motor rawTop1(1);
NoU_Motor rawBottom1(4);
NoU_Motor rawTop2(2);
NoU_Motor rawBottom2(3);

std::vector<double> top1Constants = {1, 0, 0};
std::vector<double> bottom1Constants = {1, 0, 0};
std::vector<double> top2Constants = {1, 0, 0};
std::vector<double> bottom2Constants = {1, 0, 0};

Motor top1 = Motor(1, 4, 5, top1Constants, &rawTop1);
Motor bottom1 = Motor(4, 2, 35, bottom1Constants, &rawBottom1);
Motor top2 = Motor(2, TX, RX, top2Constants, &rawTop2);
Motor bottom2 = Motor(3, 36, 39, bottom2Constants, &rawBottom2);

Module right(&top1, &bottom1, RIGHT);
Module left(&top2, &bottom2, LEFT);

// Motor top1(1, 4, 5, top1Constants, rawTop1);
// Motor bottom1(4, 2, 35, bottom1Constants, rawBottom1);
// Motor top2(2, TX, RX, top2Constants, rawTop2);
// Motor bottom2(3, 34, 39, bottom2Constants, rawBottom2);

// Module left(top1, bottom1);
// Module right(top2, bottom2);

// Drivetrain drivetrain(left, right);

double start = 0;

void top1updateEncoder() {
  int MSB = digitalRead(top1.enc1);
  int LSB = digitalRead(top1.enc2);

  int increment = 0;

  int encoded = (MSB << 1) | LSB;
  int sum = (top1.lastEncoded << 2) | encoded;

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
    top1.encoderValue--;
    top1.increment = -1;
  };
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
    top1.encoderValue++;
    top1.increment = 1;
  };

  top1.lastEncoded = encoded;

  // long currT = micros();
  // float deltaT = ((float) (currT - top1.lastTime)) / 1.0e6;
  // top1.encoderVelocity = increment/deltaT;
  // top1.lastTime = currT;
}

void bottom1updateEncoder() {
  int MSB = digitalRead(bottom1.enc1);
  int LSB = digitalRead(bottom1.enc2);

  int increment = 0;

  int encoded = (MSB << 1) | LSB;
  int sum = (bottom1.lastEncoded << 2) | encoded;

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
    bottom1.encoderValue--;
    increment = -1;
  };
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
    bottom1.encoderValue++;
    increment = 1;
  };

  bottom1.lastEncoded = encoded;
}

void top2updateEncoder() {
  int MSB = digitalRead(top2.enc1);
  int LSB = digitalRead(top2.enc2);

  int increment = 0;

  int encoded = (MSB << 1) | LSB;
  int sum = (top2.lastEncoded << 2) | encoded;

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
    top2.encoderValue--;
    increment = -1;
  };
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
    top2.encoderValue++;
    increment = 1;
  };

  top2.lastEncoded = encoded;
}

void bottom2updateEncoder() {
  int MSB = digitalRead(bottom2.enc1);
  int LSB = digitalRead(bottom2.enc2);

  int increment = 0;

  int encoded = (MSB << 1) | LSB;
  int sum = (bottom2.lastEncoded << 2) | encoded;

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
    bottom2.encoderValue--;
    increment = -1;
  };
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
    bottom2.encoderValue++;
    increment = 1;
  };

  bottom2.lastEncoded = encoded;
}

void setup() {
  Serial.begin(9600);
  char *localName = "DiffySwervePesto";
  PestoLink.begin(localName);
  // drivetrain.begin();

  attachInterrupt(digitalPinToInterrupt(top1.enc1), top1updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(top1.enc2), top1updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(bottom1.enc1), bottom1updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(bottom1.enc2), bottom1updateEncoder, CHANGE);

  // attachInterrupt(digitalPinToInterrupt(top2.enc1), top2updateEncoder, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(top2.enc2), top2updateEncoder, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(bottom2.enc1), bottom2updateEncoder, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(bottom2.enc2), bottom2updateEncoder, CHANGE);

  top1.begin();
}

void loop() {
  right.setDesiredState(moduleState(0, 90.0));
}