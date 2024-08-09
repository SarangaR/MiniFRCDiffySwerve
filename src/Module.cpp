#include <Module.h>

Module::Module(Motor* top, Motor* bottom, moduleID id) :
    top(top),
    bottom(bottom),
    id(id)
{}

void Module::setDesiredState(moduleState state) {
    double angleSpeed = getMotorSpeedsForAngle(state.angle);
    double speedSpeed = getMotorSpeedsForSpeed(state.speed);

    // Serial.println(angleSpeed);

    double top1 = angleSpeed + speedSpeed;
    double bottom1 = angleSpeed - speedSpeed;

    top->setVelocity(top1);
    bottom->setVelocity(bottom1);
}

void Module::stop() {
    top->setVelocity(0);
    bottom->setVelocity(0);
}

void Module::begin() {
    top->begin();
    bottom->begin();
    bottom->setInverted(true);

    trapezoidalProfile.init();
    trapezoidalProfile.setInitPosition(0);
}

void Module::loop() {
    top->loop();
    bottom->loop();
}

double Module::getModuleOrientation() {
    double topAngleRads = top->getPosition();
    double bottomAngleRads = bottom->getPosition();

    double angleInRads = (topAngleRads - bottomAngleRads) / 2;

    double angleInDegrees = angleInRads * 180 / PI;

    double finalAngle = angleInDegrees * gearRatio;

    // wrap the angle 0 to 360
    finalAngle = wrap0To360(finalAngle);

    return angleInDegrees;
}

double Module::getProfileState() {
    return profilePos;
}

double Module::getMotorSpeedsForAngle(double angleDegrees) {
    profilePos = trapezoidalProfile.update(angleDegrees);
    double velocityRadPerSec = trapezoidalProfile.getVelocity();

    if (trapezoidalProfile.getFinished()) {
        return 0;
    }

    return velocityRadPerSec;
}

double Module::getMotorSpeedsForSpeed(double speedMetersPerSecond) {
    double wheelCircumference = 2 * PI * wheelRadius;
    double speedRadsPerSec = speedMetersPerSecond / wheelCircumference;
    double finalSpeed = speedRadsPerSec / gearRatio;

    return finalSpeed;
}

moduleState Module::optimize(moduleState desiredState) {
    moduleState newState = moduleState(0, 0); 
    float delta = getState().angle - desiredState.angle;
    if (fabs(delta) > 90.0) {
        newState.speed = -desiredState.speed;
        newState.angle = rotateAngleBy(getState().angle, 180);
    }
    else {
        newState.speed = desiredState.speed;
        newState.angle = desiredState.angle;
    }
    return newState;
}

moduleState Module::getState() {
    double angle = getModuleOrientation();
    double gearRatio = 0.4;

    double topSpeed = top->getVelocityRads();
    double bottomSpeed = bottom->getVelocityRads();

    double speed = (topSpeed + bottomSpeed) / 2;

    speed *= gearRatio;

    return moduleState(speed, angle);
}

double Module::rotateAngleBy(double angle, double angleToRotateBy) {
    double newAngle = angle + angleToRotateBy;
    newAngle = wrap0To360(newAngle);
    return newAngle;
}

double Module::wrap0To360(double angle) {
    if (angle < 0) {
        angle += 360;
    }
    else if (angle >= 360) {
        angle -= 360;
    }
    return angle;
}

double Module::wrapNeg180To180(double angle) {
    if (angle > 180) {
        angle -= 360;
    }
    else if (angle < -180) {
        angle += 360;
    }
    return angle;
}

double Module::getError(double degrees) {
    return degrees - getModuleOrientation();
}