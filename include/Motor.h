#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include <vector>
#include <chrono>
#include <Alfredo_NoU2.h>
#include <SimpleFOC.h>
#include <SimpleDCMotor.h>
#include <SimpleFOCDrivers.h>

class Motor {
    public:
        Motor(int enc1, int enc2, std::vector<double> pidConstants, DCMotor* rawMotor, DCDriver2PWM* driver, Encoder* encoder);
        void setVelocity(double velocityRads);
        void stop();
        double getVelocityRads();
        double getPosition();
        void setInverted(bool inverted);
        void begin();
        void loop();
        void moveToPosition(double position);

        bool inverted = false;

        int enc1;
        int enc2;

        DCMotor* rawMotor;
        DCDriver2PWM* driver;
        Encoder* sensor;
    private:
        MotionControlType controlType = MotionControlType::velocity;
        std::vector<double> pidConstants;
};

#endif // MOTOR_H