#include <Motor.h>

Motor::Motor(int enc1, int enc2, NoU_Motor* rawMotor, Encoder* sensor) :
    enc1(enc1), enc2(enc2), rawMotor(rawMotor), sensor(sensor)
{}

void Motor::begin() {
    rawMotor->setDeadband(0.1);
    rawMotor->setExponent(1);
    sensor->quadrature = Quadrature::OFF;
    lastUpdate = millis();
}

void Motor::loop() {
    if (millis() - lastUpdate > 10) {  // Update encoder every 50ms (for example)
        sensor->update();
        lastUpdate = millis();
    }
}

void Motor::setVelocity(Angle velocity) {
    int dir = 1;
    if (inverted) {
        dir = -1;
    }
    float setpoint = dir * velocity.getRadians();

    rawMotor->set(setpoint/MAX_SPEED.getRadians());
}

void Motor::setInverted(bool isInverted) {
    inverted = isInverted;
}

void Motor::stop() {
    setVelocity(Angle(0));
}

Angle Motor::getPosition() {
    // return sensor->getPreciseAngle();
    return Angle(sensor->getPreciseAngle());
}

Angle Motor::getVelocity() {
    // return sensor->getVelocity();
    return Angle(sensor->getVelocity());
}