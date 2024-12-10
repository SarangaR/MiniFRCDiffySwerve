#include <Module.h>

Module::Module(Motor* top, Motor* bottom, moduleID id) :
    top(top),
    bottom(bottom),
    id(id)
{}

void Module::setDesiredState(moduleState state) {
    float angleSpeed = getMotorSpeedsForAngle(state.angle);
    float speedSpeed = getMotorSpeedsForSpeed(state.speed);

    Angle top1 = Angle(angleSpeed + speedSpeed);
    Angle bottom1 = Angle(-(angleSpeed - speedSpeed));

    top->setVelocity(top1);
    bottom->setVelocity(bottom1);
}

void Module::stop() {
    top->setVelocity(Angle(0));
    bottom->setVelocity(Angle(0));
}

void Module::begin() {
    top->begin();
    bottom->begin();
    top->setInverted(false);
    pid.reset();
    bottom->setInverted(true);
}

void Module::loop() {
    top->loop();
    bottom->loop();
}

Angle Module::getModuleOrientation() {
    float topMotorAngle = top->getPosition().getRadians();
    float bottomMotorAngle = bottom->getPosition().getRadians();

    float topModuleAngle = -topMotorAngle * gearRatioTurn;
    float bottomModuleAngle = bottomMotorAngle * gearRatioTurn;

    Angle angle = Angle((topModuleAngle + bottomModuleAngle) / 2, RADIANS);

    return angle.wrapNeg180To180();
}

float Module::getModuleSpeed() {
    float topMotorSpeed = top->getVelocity().getRadians();
    float bottomMotorSpeed = bottom->getVelocity().getRadians();

    float topSpeed = topMotorSpeed * gearRatioSpin;
    float bottomSpeed = bottomMotorSpeed * gearRatioSpin;

    Angle speed = Angle((topSpeed - bottomSpeed) / 2);

    //convert from rad/s to m/s
    float speedMetersPerSecond = speed.getRadians() * wheelRadius;

    return speedMetersPerSecond;
}

float Module::getProfileState() {
    return profilePos;
}

float Module::getMotorSpeedsForAngle(float angleDegrees) {
    angleTarget = angleDegrees;
    float error = angleDegrees - getModuleOrientation().getDegrees();
    float pidOutput = pid(error);
    
    return pidOutput / gearRatioTurn;
}

float Module::getMotorSpeedsForSpeed(float speedMetersPerSecond) {
    float speedRadsPerSec = speedMetersPerSecond / wheelRadius;
    float finalSpeed = speedRadsPerSec / gearRatioSpin;

    speedTarget = finalSpeed;

    return finalSpeed;
}

moduleState Module::getState() {
    float angle = getModuleOrientation().getDegrees();
    float speed = getModuleSpeed();

    return moduleState(speed, angle);
}

float Module::rotateAngleBy(float angle, float angleToRotateBy) {
    float newAngle = angle + angleToRotateBy;
    return Angle(newAngle, DEGREES).wrap().getDegrees();
}

float Module::wrapNeg180To180(float angle) {
    if (angle > 180) {
        angle -= 360;
    }
    else if (angle < -180) {
        angle += 360;
    }
    return angle;
}

float Module::getError(float degrees) {
    return degrees - getModuleOrientation().getDegrees();
}

float Module::getErrorModifier(float expected, float actual) {
    float modifier = expected / actual;
    return modifier;
}

void Module::setMotorInvert(bool topInvert, bool bottomInvert) {
    top->setInverted(topInvert);
    bottom->setInverted(bottomInvert);
}

void Module::setBrake(bool brake) {
    if (brake) {
        top->setBrake(BRAKE);
        bottom->setBrake(BRAKE);
    } else {
        top->setBrake(RELEASE);
        bottom->setBrake(RELEASE);
    }
}

