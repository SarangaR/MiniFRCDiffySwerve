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

std::array<moduleState, 3> Drivetrain::toSwerveModuleStates(float vxf, float vyf, float omega, Angle angle, bool fieldOriented, HWCDC *serial) {
    float gyroAngle = fieldOriented ? angle.wrapNeg180To180().getRadians() : 0;

    // Rotate velocities by gyro angle if in field-oriented mode
    float vx = vxf * cos(gyroAngle) + vyf * sin(gyroAngle);
    float vy = -vxf * sin(gyroAngle) + vyf * cos(gyroAngle);

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
        float moduleVx = vx - omega * moduleY;
        float moduleVy = vy + omega * moduleX;

        // Compute speed and angle for each module
        float speed = hypot(moduleVx, moduleVy); // Calculate the module's speed
        Angle moduleAngle = (speed > 1e-6) ? Angle(moduleVx, moduleVy) : getModuleOrientations()[i];

        // Set module state with speed and angle
        moduleStates[i] = moduleState(speed, moduleAngle.wrapNeg180To180().getDegrees());
    }

    // moduleStates = optimize(moduleStates, getModuleStates());
    // moduleStates = normalizeSpeeds(moduleStates);

    return moduleStates;
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
            newState.angle = Angle(desiredState.angle + 180.0f, DEGREES).wrapNeg180To180().getDegrees();
        }
        else {
            newState.speed = desiredState.speed;
            newState.angle = desiredState.angle;
        }
        newStates[i] = newState;
    }
    return newStates;
} 

std::array<moduleState, 3> Drivetrain::drive(float vx, float vy, float omega, Angle gyroAngle, bool fieldOriented, HWCDC *serial) {
    std::array<moduleState, 3> states = toSwerveModuleStates(vx, vy, omega, gyroAngle, fieldOriented, serial);

    left->setDesiredState(states[0]);
    right->setDesiredState(states[1]);
    center->setDesiredState(states[2]);

    return states;
}

std::array<moduleState, 3> Drivetrain::normalizeSpeeds(std::array<moduleState, 3> states) {
    std::array<float, 3> speeds = {0, 0, 0};
    float realMaxSpeed = 0;
    for (moduleState state : states) {
        realMaxSpeed = fmax(realMaxSpeed, fabs(state.speed));
    }

    if (realMaxSpeed > Module::MAX_SPEED_SPIN_MS) {
        for (int i = 0; i < states.size(); i++) {
            states[i].speed = states[i].speed / realMaxSpeed * Module::MAX_SPEED_SPIN_MS;
        }
    }
}

std::array<Angle, 3> Drivetrain::getModuleOrientations() {
    return {left->getModuleOrientation(), right->getModuleOrientation(), center->getModuleOrientation()};
}

std::array<float, 3> Drivetrain::getModuleSpeeds() {
    return {left->getModuleSpeed(), right->getModuleSpeed(), center->getModuleSpeed()};
}

std::array<moduleState, 3> Drivetrain::getModuleStates() {
    return {left->getState(), right->getState(), center->getState()};
}

void Drivetrain::stop() {
    right->stop();
    left->stop();
    center->stop();
}
