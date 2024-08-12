#include <Motor.h>

Motor::Motor(int enc1, int enc2, std::vector<double> pidConstants, DCMotor* rawMotor, DCDriver2PWM* driver, Encoder* sensor) :
    enc1(enc1), enc2(enc2), pidConstants(pidConstants), rawMotor(rawMotor), driver(driver), sensor(sensor)
{}

void Motor::begin() {
    driver->voltage_power_supply = 9.0;
    driver->voltage_limit = 9.0;
    driver->pwm_frequency = MOTOR_PWM_FREQ;
    driver->init();

    rawMotor->linkDriver(driver);
    rawMotor->linkSensor(sensor);

    rawMotor->voltage_limit = 9.0;
    rawMotor->velocity_limit = 500.0;

    rawMotor->controller = controlType;
    rawMotor->torque_controller = TorqueControlType::voltage;

    rawMotor->init();
    rawMotor->initFOC();

    rawMotor->foc_modulation = FOCModulationType::SpaceVectorPWM;

    rawMotor->PID_velocity.P = pidConstants[0];
    rawMotor->PID_velocity.I = pidConstants[1];
    rawMotor->PID_velocity.D = pidConstants[2];

    rawMotor->P_angle.P = 10.0;
    rawMotor->P_angle.I = 0.0;
    rawMotor->P_angle.D = 0.0;

    rawMotor->PID_velocity.output_ramp = 1000.0;
    rawMotor->LPF_velocity.Tf = 0.01;
    rawMotor->LPF_angle.Tf = 0.0;

    rawMotor->target = 0;

    rawMotor->enable();

    // Serial.println("Motor Initialized");
}

void Motor::loop() {
    sensor->update();
    rawMotor->loopFOC();
}

void Motor::setVelocity(double velocity) {
    if (controlType != MotionControlType::velocity) {
        controlType = MotionControlType::velocity;
        rawMotor->controller = controlType;
    }

    int dir = 1;
    if (inverted) {
        dir = -1;
    }

    double min_vel = 0.2*26;

    if (fabs(velocity) < min_vel) {
        velocity = 0;
    }
    rawMotor->move(velocity*dir);
}

void Motor::setInverted(bool isInverted) {
    inverted = isInverted;
}

void Motor::stop() {
    setVelocity(0);
}

double Motor::getPosition() {
    // return sensor->getPreciseAngle();
    return rawMotor->shaftAngle();;
}

double Motor::getVelocityRads() {
    // return sensor->getVelocity();
    return rawMotor->shaftVelocity();
}

void Motor::moveToPosition(double position) {
    if (controlType != MotionControlType::angle) {
        controlType = MotionControlType::angle;
        rawMotor->controller = controlType;
    }

    rawMotor->move(position);
}