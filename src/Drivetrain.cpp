#include <Drivetrain.h>

Drivetrain::Drivetrain(Module* left, Module* right) :
    left(left),
    right(right)
{}

void Drivetrain::begin() {
    left->begin();
    right->begin();
}

double Drivetrain::getGyroAngle() {
    //TODO: Implement this
    double gyroAngle = 0;

    return right->wrapNeg180To180(gyroAngle);
}

void Drivetrain::loop() {
    left->loop();
    right->loop();
}

std::vector<moduleState> Drivetrain::toSwerveModuleStates(double vxf, double vyf, double omega) {
    double gyroAngle = getGyroAngle();
    double temp = vyf * cos(gyroAngle) + vxf * sin(gyroAngle);
    double vx = -vyf * sin(gyroAngle) + vxf * cos(gyroAngle);
    double vy = temp;

    double R = sqrt(pow(LENGTH, 2) + pow(WIDTH, 2));

    double A = vx - omega * (LENGTH/R);
    double B = vx  + omega * (LENGTH/R);
    double C = vy - omega * (WIDTH/R);
    double D = vy + omega * (WIDTH/R);

    double frSpeed = sqrt(pow(B, 2) + pow(C, 2));
    double flSpeed = sqrt(pow(B, 2) + pow(D, 2));
    double blSpeed = sqrt(pow(A, 2) + pow(D, 2));
    double brSpeed = sqrt(pow(A, 2) + pow(C, 2));

    double frAngle = atan2(B, C) * 180 / PI;
    double flAngle = atan2(B, D) * 180 / PI;
    double blAngle = atan2(A, D) * 180 / PI;
    double brAngle = atan2(A, C) * 180 / PI;

    frAngle = right->wrapNeg180To180(frAngle);
    flAngle = right->wrapNeg180To180(flAngle);
    blAngle = right->wrapNeg180To180(blAngle);
    brAngle = right->wrapNeg180To180(brAngle);

    std::vector<double> speeds = {frSpeed, blSpeed};
    speeds = normalizeSpeeds(speeds);

    double frSpeedNormalized = speeds[0];
    double blSpeedNormalized = speeds[1];

    moduleState FR = {frSpeedNormalized, frAngle};
    moduleState BL = {blSpeedNormalized, -blAngle};

    moduleState optimizedFR = optimize(FR, right->getState());
    moduleState optimizedBL = optimize(BL, left->getState());

    return {optimizedFR, optimizedBL};
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

std::vector<moduleState> Drivetrain::drive(double vx, double vy, double omega) {
    std::vector<moduleState> states = toSwerveModuleStates(vx, vy, omega);

    lastModuleStates = {states[0].toString(), states[1].toString()};

    right->setDesiredState(states[0]);
    left->setDesiredState(states[1]);

    return states;
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
    left->stop();
}
