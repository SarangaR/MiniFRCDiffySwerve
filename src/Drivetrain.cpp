#include <Drivetrain.h>

Drivetrain::Drivetrain(Module* left, Module* right) :
    left(left),
    right(right)
{}

void Drivetrain::begin() {
    // left->begin();
    right->begin();
}

double Drivetrain::getGyroAngle() {
    //TODO: Implement this
    return 0;
}

void Drivetrain::loop() {
    // left->loop();
    right->loop();
}

std::vector<moduleState> Drivetrain::toSwerveModuleStates(double vxf, double vyf, double omega) {
    double gyroAngle = getGyroAngle();
    double vx = vxf * cos(gyroAngle) + vyf * sin(gyroAngle);
    double vy = -vxf * sin(gyroAngle) + vyf * cos(gyroAngle);

    double A = vx - omega * (LENGTH/2.0);
    double B = vx  + omega * (LENGTH/2.0);
    double C = vy - omega * (WIDTH/2.0);
    double D = vy + omega * (WIDTH/2.0);

    double topRightSpeed = sqrt(pow(B, 2) + pow(C, 2));
    double topRightAngle = atan2(B, C) * 180 / PI;

    double bottomLeftSpeed = sqrt(pow(A, 2) + pow(D, 2));
    double bottomLeftAngle = atan2(A, D) * 180 / PI;

    std::vector<double> speeds = {topRightSpeed, bottomLeftSpeed};
    speeds = normalizeSpeeds(speeds);

    double topRightSpeedNormalized = speeds[0];
    double bottomLeftSpeedNormalized = speeds[1];

    moduleState topRight = {topRightSpeedNormalized, topRightAngle};
    moduleState bottomLeft = {bottomLeftSpeedNormalized, bottomLeftAngle};

    moduleState optimizedTopRight = optimize(topRight, right->getState());
    moduleState optimizedBottomLeft = optimize(bottomLeft, left->getState());

    return {optimizedTopRight, optimizedBottomLeft};
}

moduleState Drivetrain::optimize(moduleState desiredState, moduleState currentState) {
    moduleState newState = moduleState(0, 0); 
    float delta = currentState.angle - desiredState.angle;
    if (fabs(delta) > 90.0) {
        newState.speed = -desiredState.speed;
        newState.angle = right->wrapNeg180To180(desiredState.angle + 180.0);
    }
    else {
        newState.speed = desiredState.speed;
        newState.angle = desiredState.angle;
    }
    return newState;
}

void Drivetrain::drive(double vx, double vy, double omega) {
    std::vector<moduleState> states = toSwerveModuleStates(vx, vy, omega);

    right->setDesiredState(states[0]);
    // left->setDesiredState(states[1]);
}

std::vector<double> Drivetrain::normalizeSpeeds(std::vector<double> speeds) {
    double max = *std::max_element(speeds.begin(), speeds.end());
    if (max > 1) {
        for (int i = 0; i < speeds.size(); i++) {
            speeds[i] /= max;
        }
    }

    return speeds;
}

void Drivetrain::stop() {
    right->stop();
    // left->stop();
}
