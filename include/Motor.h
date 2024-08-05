#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include <PIDController.h>
#include <vector>
#include <chrono>
#include <Alfredo_NoU2.h>

class Motor {
    public:
        Motor(int port, int enc1, int enc2, std::vector<double> pidConstants, NoU_Motor* rawMotor);
        void setVelocity(double velocityRPM);
        void stop();
        double getVelocityRPM();
        double getPosition();
        void resetPosition();
        void setInverted(bool inverted);
        void begin();
        void setPower(double power);
        void setPWM(int dir, int pwmVal, int pwm, int in1, int in2);
        double RPMtoTicksPerSecond(double RPM);
        double TicksPerSecondtoRPM(double TPS);
        void moveRotationsAbsolute(double rotations);
        void moveRotationsRelative(double rotations);

        bool rotationFlag = false;


        double encoderValue = 0;
        double encoderVelocity = 0;
        double prevVelocity = 0;
        double vFilter = 0;
        int lastEncoded = 0;
        int lastEncoderValue = 0;
        double prevErrorVelocity = 0;
        double prevErrorPosition = 0;

        double positionTarget = 0;

        double lastTime = 0;

        int port;
        int enc1;
        int enc2;

        int increment = 0;

        NoU_Motor* rawMotor;
    private:
        PIDController pid;
        std::vector<double> pidConstants;

        double TICKS_PER_MODULE_REV = 1100;
};

#endif // MOTOR_H