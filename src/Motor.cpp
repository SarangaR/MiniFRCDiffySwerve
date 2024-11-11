#include <Motor.h>

Motor::Motor(int enc1, int enc2, std::vector<float> pidConstants, NoU_Motor* rawMotor, Encoder* sensor) :
    enc1(enc1), enc2(enc2), pidConstants(pidConstants), rawMotor(rawMotor), sensor(sensor)
{}

void Motor::begin() {
    rawMotor->setDeadband(0.1);
    rawMotor->setExponent(1);
    sensor->quadrature = Quadrature::OFF;
}

void Motor::loop() {
    sensor->update();
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