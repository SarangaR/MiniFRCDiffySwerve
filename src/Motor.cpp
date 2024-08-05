#include <Motor.h>

Motor::Motor(int port, int enc1, int enc2, std::vector<double> pidConstants, NoU_Motor* rawMotor) :
    port(port), enc1(enc1), enc2(enc2), pidConstants(pidConstants), rawMotor(rawMotor)
{}

void Motor::begin() {
    pid.begin();
    pid.tune(pidConstants[0], pidConstants[1], pidConstants[2]);
}

void Motor::setVelocity(double velocityRPM) {
    double velocityTPS = velocityRPM * TICKS_PER_MODULE_REV / 60;
    double currVel = getVelocityRPM();

    double kp = 1;
    double kd = 0.5;
    double error = velocityTPS - currVel;
    double u = kp * error;

    if (u >= 7140) {
        u = 7140;
    }
    else if (u <= -7140) {
        u = -7140;
    }
    // Serial.println(">OutputRaw:" + String(u));
    u = u/7140;
    // Serial.println(">OutputPre:" + String(u));
    if (fabs(u) < 0.03) {
        u = 0;
    }
    else if (fabs(u) < 0.75) {
        if (u > 0) {
            u = 0.75;
        }
        else if (u < 0){
            u = -0.75;
        }
    }

    // Serial.println(">Error:" + String(error));
    // Serial.println(">Output:" + String(u));
    // setPWM(dir, pwr, pwm, aPin, bPin);
    rawMotor->set(u);
}

void Motor::moveRotationsAbsolute(double rotations) {
    double target = rotations * TICKS_PER_MODULE_REV;
    double error = target - getPosition();
    double kp = 0.5;
    double u = kp * error;
    Serial.println(">Output:" + String(u));
    Serial.println(">Error:" + String(error));
    if (fabs(error) < 5) {
        setPower(0);
    }
    else {
        setVelocity(TicksPerSecondtoRPM(u));
    }
}

void Motor::moveRotationsRelative(double rotations) {
    if (!rotationFlag) {
        positionTarget = getPosition() + rotations * TICKS_PER_MODULE_REV;
        rotationFlag = true;
    }
    double error = positionTarget - getPosition();
    double kp = 0.5;
    double u = kp * error;
    Serial.println(">Output:" + String(u));
    Serial.println(">Error:" + String(error));
    if (fabs(error) < 5) {
        rotationFlag = false;
        setPower(0);
    }
    else {
        if (u >= 7140) {
            u = 7140;
        }
        else if (u <= -7140) {
            u = -7140;
        }
        // Serial.println(">OutputRaw:" + String(u));
        u = u/7140;
        // Serial.println(">OutputPre:" + String(u));
        if (fabs(u) < 0.03) {
            u = 0;
        }
        else if (fabs(u) < 0.75) {
            if (u > 0) {
                u = 0.75;
            }
            else if (u < 0){
                u = -0.75;
            }
        }
        setPower(u);
    }
}

void Motor::setPWM(int dir, int pwmVal, int pwm, int in1, int in2) {
    analogWrite(pwm, pwmVal);
    if (dir == 1) {
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
    } else if (dir == -1) {
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
    }
    else {
        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW);
    }
}

void Motor::setPower(double power) {
    rawMotor->set(power);
}

void Motor::setInverted(bool inverted) {
    rawMotor->setInverted(inverted);
}

void Motor::stop() {
    setVelocity(0);
}

double Motor::getPosition() {
    return -encoderValue;
}

void Motor::resetPosition() {
    encoderValue = 0;
}

double Motor::getVelocityRPM() {
    // double velocityTPS = encoderVelocity;
    // double rpm = (velocityTPS / TICKS_PER_MODULE_REV) * 60;  

    long currT = micros();
    float deltaT = ((float) (currT - lastTime)) / 1.0e6;
    double velocityTPS = (encoderValue - lastEncoderValue) / deltaT;
    lastEncoderValue = encoderValue;
    lastTime = currT;

    // double rpm = (velocityTPS / TICKS_PER_MODULE_REV) * 60;

    // vFilter = 0.854*vFilter + 0.0728*velocityTPS + 0.0728*prevVelocity;
    // prevVelocity = velocityTPS;

    return -velocityTPS;
}

double Motor::RPMtoTicksPerSecond(double RPM) {
    return RPM * TICKS_PER_MODULE_REV / 60;
}

double Motor::TicksPerSecondtoRPM(double TPS) {
    return TPS * 60 / TICKS_PER_MODULE_REV;
}