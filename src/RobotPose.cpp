#include <RobotPose.h>

RobotPose::RobotPose() {}

bool RobotPose::begin(float magnetic_declination) {
    mag_dec = magnetic_declination;
    if (!otos.begin()) {
        return false;
    }
    otos.setLinearScalar(1.0);
    otos.setAngularScalar(1.0);
    otos.setLinearUnit(kSfeOtosLinearUnitMeters);
    otos.setAngularUnit(kSfeOtosAngularUnitDegrees);
    otos.calibrateImu();
    otos.resetTracking();
    lastHeadingUpdate = millis();
    return true;
}

void RobotPose::setYawOffset(float offset) {
    yaw_offset = offset;
}

void RobotPose::setYawOffset(float mag_x, float mag_y) {
    yaw_offset = degrees(atan2(mag_y, mag_x)) + mag_dec;
}

void RobotPose::updateHeading(float mag_x, float mag_y) {
    otos.getPosition(robotPose);
    robotPose.x = xFilter(robotPose.x);
    robotPose.y = yFilter(robotPose.y);
    float yaw1 = -robotPose.h;

    float yaw2 = degrees(atan2(mag_y, mag_x)) + mag_dec;
    yaw2 -= yaw_offset;

    if (yaw2 > 180.0f) {
        yaw2 -= 360.0f;
    }
    else if (yaw2 < -180.0f) {
        yaw2 += 360.0f;
    }

    currentYaw = (yaw1 * sensor_imu_weight) + (yaw2 * mag_weight);
    currentYaw = hFilter(currentYaw);
    currentYaw *= angularScalar;
    currentYaw = Angle(currentYaw, DEGREES).wrapNeg180To180().getDegrees();
    lastHeadingUpdate = millis();
}

Angle RobotPose::getHeading() {
    return Angle(currentYaw, DEGREES);
}

void RobotPose::calibrateHeading() {
    setYawOffset(currentYaw);
    otos.calibrateImu();
}

void RobotPose::resetTracking() {
    otos.resetTracking();
}

void RobotPose::setUnits(sfe_otos_linear_unit_t linearUnit, sfe_otos_angular_unit_t angularUnit) {
    otos.setLinearUnit(linearUnit);
    otos.setAngularUnit(angularUnit);
}

sfe_otos_pose2d_t RobotPose::getPosition() {
    otos.getPosition(robotPose);
    robotPose.x = xFilter(robotPose.x);
    robotPose.y = yFilter(robotPose.y);
    robotPose.h = getHeading().getDegrees();
    return robotPose;
}

Angle RobotPose::getOTOSHeading() {
    return Angle(hFilter2(robotPose.h), DEGREES);
}

void RobotPose::updateOTOS() {
    otos.getPosition(robotPose);
}

Angle RobotPose::getMagHeading(float mx, float my) {
    float yaw = degrees(atan2(my, mx)) + mag_dec;
    yaw -= yaw_offset;

    if (yaw > 180.0f) {
        yaw -= 360.0f;
    }
    else if (yaw < -180.0f) {
        yaw += 360.0f;
    }
    return Angle(yaw, DEGREES).wrapNeg180To180();
}

void RobotPose::update(float mag_x, float mag_y) {
    updateHeading(mag_x, mag_y);
    getPosition();
}

void RobotPose::setOTOSLinearScalar(float scalar) {
    otos.setLinearScalar(scalar);
}

void RobotPose::setAngularScalar(float scalar) {
    angularScalar = scalar;
}

void RobotPose::setOTOSAngularScalar(float scalar) {
    otos.setAngularScalar(scalar);
}