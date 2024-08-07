#include <Module.h>

Module::Module(Motor* top, Motor* bottom, moduleID id) :
    top(top),
    bottom(bottom),
    id(id)
{}

void Module::setDesiredState(moduleState state) {
    // moduleState newState = optimize(state);
    double angleSpeed = getMotorSpeedsForAngle(state.angle);
    double speedSpeed = getMotorSpeedsForSpeed(state.speed);

    double top1 = angleSpeed + speedSpeed;
    double bottom1 = -(angleSpeed + speedSpeed);

    top->setVelocity(top1);
    bottom->setVelocity(bottom1);
}

void Module::stop() {
    top->setPower(0);
    bottom->setPower(0);
}

void Module::begin() {
    top->begin();
    bottom->begin();
}

double Module::getModuleOrientation() {
    double topPosition = top->getPosition();
    double bottomPosition = bottom->getPosition();

    double angle = (topPosition - bottomPosition) / 16;

    //convert angle to degrees
    double ticksPerRevolution = 1100;//1440;
    double gearRatio = 0.4;

    double ticksPerModuleRev = ticksPerRevolution * gearRatio;

    double angleInDegrees = (angle / ticksPerModuleRev) * 360;

    // wrap the angle 0 to 360
    angleInDegrees = wrap0To360(angleInDegrees);

    // if (angleInDegrees > 180) {
    //     angleInDegrees -= 360;
    // } else if (angleInDegrees < -180) {
    //     angleInDegrees += 360;
    // }

    return angleInDegrees;
}

double Module::getMotorSpeedsForAngle(double angleDegrees) {
    double kp = 0.3;
    double kd = 0.1;
    double error = angleDegrees - getModuleOrientation();
    double u = kp * error + kd * (error - prevErrorAngle);
    prevErrorAngle = error;

    // convert u from degreesPerSecond to ticksPerSecond
    double ticksPerRevolution = 1100;//1440;
    double gearRatio = 0.4;

    double ticksPerModuleRev = ticksPerRevolution * gearRatio;

    double ticksPerSecond = (u / 360) * ticksPerModuleRev;

    Serial.println(error);

    if (fabs(error) < 3) {
        return 0;
    }

    return ticksPerSecond;
}

double Module::getMotorSpeedsForSpeed(double speed) {
    double ticksPerRevolution = 1100;//1440;
    double gearRatio = 0.4;

    double ticksPerModuleRev = ticksPerRevolution * gearRatio;

    double ticksPerSecond = (speed / 360) * ticksPerModuleRev;

    return ticksPerSecond;
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

    double topSpeed = top->getVelocityRPM();
    double bottomSpeed = bottom->getVelocityRPM();

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

void Module::resetState() {
    top->resetPosition();
    bottom->resetPosition();
}

double Module::getError(double degrees) {
    return degrees - getModuleOrientation();
}