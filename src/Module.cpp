#include <Module.h>

Module::Module(Motor* top, Motor* bottom, moduleID id) :
    top(top),
    bottom(bottom),
    id(id)
{}

void Module::setDesiredState(moduleState state) {
    double angleSpeed = getMotorSpeedsForAngle(state.angle);
    double speedSpeed = getMotorSpeedsForSpeed(state.speed);

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

    if (id == LEFT) {
        angleInDegrees *= -1;
    }

    double errorModifierRight = getErrorModifier(90, 74.53);
    double errorModifierLeft = getErrorModifier(90, 74.91);

    double finalAngle = angleInDegrees;

    switch (id) {
        case RIGHT:
            finalAngle *= errorModifierRight;
            break;
        case LEFT:
            finalAngle *= errorModifierLeft;
            break;
    }

    return wrapNeg180To180(finalAngle);
}

double Module::getModuleSpeed() {
    double topSpeed = top->getVelocityRads();
    double bottomSpeed = bottom->getVelocityRads();

    double speed = (topSpeed + bottomSpeed) / 2;

    return speed;
}

double Module::getProfileState() {
    return profilePos;
}

double Module::getMotorSpeedsForAngle(double angleDegrees) {
    angleTarget = angleDegrees;
    double error = angleDegrees - getModuleOrientation();
    double pidOutput = pid(error);

    if (fabs(pidOutput) > 0.2*26) {
        return pidOutput;
    }
    else {
        return 0;
    }
}

double Module::getMotorSpeedsForSpeed(double speedMetersPerSecond) {
    double wheelCircumference = 2 * PI * wheelRadius;
    double speedRadsPerSec = speedMetersPerSecond / wheelCircumference;
    double finalSpeed = speedRadsPerSec / gearRatio;

    speedTarget = finalSpeed;

    return finalSpeed;
}

moduleState Module::getState() {
    double angle = getModuleOrientation();
    double speed = getModuleSpeed();

    return moduleState(speed, angle);
}

double Module::rotateAngleBy(double angle, double angleToRotateBy) {
    double newAngle = angle + angleToRotateBy;
    return wrapNeg180To180(newAngle);
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

double Module::getErrorModifier(double expected, double actual) {
    double modifier = expected / actual;
    return modifier;
}

