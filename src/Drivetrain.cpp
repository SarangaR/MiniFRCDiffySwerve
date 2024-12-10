#include <Drivetrain.h>

Drivetrain::Drivetrain(Module* left, Module* right, Module* center) :
    left(left),
    right(right),
    center(center)
{}

void Drivetrain::begin() {
    left->begin();
    right->begin();
    center->begin();

    // for (int i = 0; i < numModules; i++) {
    //     inverseKinematics.row(i*2 + 0) = Eigen::Vector3d(1, 0, -modulePositions[i].y());
    //     inverseKinematics.row(i*2 + 1) = Eigen::Vector3d(0, 1, modulePositions[i].x());
    // }
    // forwardKinematics = inverseKinematics.inverse();
}

float Drivetrain::getGyroAngle() {
    //TODO: Implement this
    Angle gyroAngle = Angle(0, DEGREES);

    return gyroAngle.wrapNeg180To180().getRadians();
}

void Drivetrain::loop() {
    left->loop();
    right->loop();
    center->loop();
}

std::array<moduleState, 3> Drivetrain::toSwerveModuleStates(float vxf, float vyf, float omega, Angle angle, bool fieldOriented) {
    Angle gyroAngle = fieldOriented ? angle : Angle(0);

    std::array<float, 3U> chassisSpeeds = fromFieldRelativeSpeeds(vxf, vyf, omega, gyroAngle);
    float vx = chassisSpeeds[0];
    float vy = chassisSpeeds[1];

    std::array<moduleState, 3> moduleStates = {
        moduleState(0, 0),
        moduleState(0, 0),
        moduleState(0, 0),
    };

    // Loop over each module to calculate its speed and angle
    for (size_t i = 0; i < numModules; i++) {
        // Module position offsets
        float moduleX = modulePositions[i].x();
        float moduleY = modulePositions[i].y();

        // Compute the velocity components for each module
        float moduleVx = vx - omega * moduleX;
        float moduleVy = vy + omega * moduleY;

        // Compute speed and angle for each module
        float speed = hypot(moduleVx, moduleVy); // Calculate the module's speed
        Angle moduleAngle = (speed > 1e-6) ? Angle(moduleVx, moduleVy) : getModuleOrientations()[i];

        // Set module state with speed and angle
        moduleStates[i] = moduleState(speed, moduleAngle.wrapNeg180To180().getDegrees());
        moduleStates[i].optimize(getModuleOrientations()[i]);

    }

    return moduleStates;
}

std::array<float, 3U> Drivetrain::fromFieldRelativeSpeeds(float vx, float vy, float omega, Angle gyroAngle) {
    float robotVx = vx * cosf(gyroAngle.getRadians()) - vy * sinf(gyroAngle.getRadians());
    float robotVy = vx * sinf(gyroAngle.getRadians()) + vy * cosf(gyroAngle.getRadians());

    return {robotVx, robotVy, omega};
}
 
std::array<moduleState, 3> Drivetrain::optimize(std::array<moduleState, 3> desiredStates, std::array<moduleState, 3> currentStates) {
    std::array<moduleState, 3> newStates = {moduleState(0, 0), moduleState(0, 0), moduleState(0, 0)};
    for (int i = 0; i < desiredStates.size(); i++) {
        moduleState desiredState = desiredStates[i];
        moduleState currentState = currentStates[i];
        moduleState newState = moduleState(0, 0); 
        Angle delta = Angle(currentState.angle, DEGREES) - Angle(desiredState.angle, DEGREES);
        if (fabs(delta.wrapNeg180To180().getDegrees()) > 90.0f) {
            newState.speed = -desiredState.speed;
            newState.angle = Angle(desiredState.angle + 180, DEGREES).wrapNeg180To180().getDegrees();
        }
        else {
            newState.speed = desiredState.speed;
            newState.angle = desiredState.angle;
        }
        newStates[i] = newState;
    }
    return newStates;
} 

std::array<moduleState, 3> Drivetrain::drive(float vx, float vy, float omega, Angle gyroAngle, bool fieldOriented) {
    std::array<moduleState, 3> states = toSwerveModuleStates(vx, vy, omega, gyroAngle, fieldOriented);

    lastModuleStates = {states[0].toString(), states[1].toString(), states[2].toString()};

    left->setDesiredState(states[0]);
    right->setDesiredState(states[1]);
    center->setDesiredState(states[2]);

    return states;
}

std::array<moduleState, 3> Drivetrain::normalizeSpeeds(std::array<moduleState, 3> states) {
    std::vector<float> speeds;
    for (int i = 0; i < states.size(); i++) {
        speeds.push_back(states[i].speed);
    }

    float max = *std::max_element(speeds.begin(), speeds.end());
    if (max > Module::MAX_SPEED_SPIN_MS) {
        for (int i = 0; i < speeds.size(); i++) {
            speeds[i] /= max;
        }
    }

    std::array<moduleState, 3> normalizedStates = {moduleState(0, 0), moduleState(0, 0), moduleState(0, 0)};

    for (int i = 0; i < states.size(); i++) {
        normalizedStates[i] = moduleState(speeds[i], states[i].angle);
    } 

    return normalizedStates;
}

std::vector<Angle> Drivetrain::getModuleOrientations() {
    return {left->getModuleOrientation(), right->getModuleOrientation(), center->getModuleOrientation()};
}

std::vector<float> Drivetrain::getModuleSpeeds() {
    return {left->getModuleSpeed(), right->getModuleSpeed(), center->getModuleSpeed()};
}

std::array<moduleState, 3> Drivetrain::getModuleStates() {
    return {left->getState(), right->getState(), center->getState()};
}

void Drivetrain::setBrake(bool brake) {
    left->setBrake(brake);
    right->setBrake(brake);
    center->setBrake(brake);
}

void Drivetrain::stop() {
    right->stop();
    left->stop();
    center->stop();
}

int Drivetrain::findClosestPoint(const std::vector<Point>& spline, float x, float y) {
    int closestIndex = 0;
    double minDistance = 1e6;

    for (int i = 0; i < spline.size(); i++) {
        double distance = sqrt(pow(spline[i].x - x, 2) + pow(spline[i].y - y, 2));
        if (distance < minDistance) {
        minDistance = distance;
        closestIndex = i;
        }
    }
    return closestIndex;
}

Point Drivetrain::findLookaheadPoint(const std::vector<Point>& spline, int closestIndex, float lookaheadDistance, sfe_otos_pose2d_t pose) {
    float currX = -pose.x;
    float currY = -pose.y;
    for (int i = closestIndex; i < spline.size(); i++) {
        double distance = sqrt(pow(spline[i].x - currX, 2) + pow(spline[i].y - currY, 2));
        if (distance >= lookaheadDistance) {
        return spline[i];
        }
    }
    return spline.back(); // Return the last point if no lookahead point is found
}

void Drivetrain::followSpline(const std::vector<Point>& spline, sfe_otos_pose2d_t pose, float LOOKAHEAD_DISTANCE, float KP_POSITION, float KP_ORIENTATION) {
    float currX = -pose.x;
    float currY = -pose.y;

    // Find the closest point on the spline
    int closestIndex = findClosestPoint(spline, currX, currY);

    // Find the lookahead point
    Point lookaheadPoint = findLookaheadPoint(spline, closestIndex, LOOKAHEAD_DISTANCE, pose);

    // Compute desired velocities
    double vx = KP_POSITION * (lookaheadPoint.x - currX);
    double vy = KP_POSITION * (lookaheadPoint.y - currY);
    double desiredTheta = atan2(lookaheadPoint.y - pose.h, lookaheadPoint.x - pose.h);
    double omega = KP_ORIENTATION * (desiredTheta - pose.h);

    // Transform velocities to robot-centric
    double vxRobot = vx * cos(pose.h) + vy * sin(pose.h);
    double vyRobot = -vx * sin(pose.h) + vy * cos(pose.h);

    // Send velocity commands to the robot
    this->drive(vxRobot, vyRobot, omega, Angle(pose.h, DEGREES), false);
}
