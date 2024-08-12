#ifndef MODULE_H
#define MODULE_H

#include <Motor.h>
#include <MotionGenerator.h>

struct moduleState {
    double speed;
    double angle;

    moduleState(double speed, double angle) :
        speed(speed),
        angle(angle)
    {}

    String toString() {
        return "Speed: " + String(speed) + " Angle: " + String(angle);
    }
};

enum moduleID {
    LEFT, RIGHT
};

class Module {
    public:
        Module(Motor* top, Motor* bottom, moduleID id);
        void setDesiredState(moduleState state);
        void stop();
        void begin();
        double getModuleOrientation();
        double getModuleSpeed();
        std::vector<double> rotateModule(double angle);
        double getMotorSpeedsForAngle(double angleDegrees);
        double getMotorSpeedsForSpeed(double speed);
        moduleState getState();
        double rotateAngleBy(double angle, double angleToRotateBy);
        double wrap0To360(double angle);
        double wrapNeg180To180(double angle);
        double getError(double degrees);
        double getProfileState();
        double getErrorModifier(double expected, double actual);

        void loop();

        moduleID id;
    private:
        Motor* top;
        Motor* bottom;

        double prevErrorAngle = 0;

        double wheelRadius = 0.05;
        double gearRatio = 0.4;

        MotionGenerator trapezoidalProfile = MotionGenerator(30, 100, 0);

        float profilePos = 0;
        double angleTarget = 0;
        double speedTarget = 0;

        bool alreadyDone = false;

        PIDController pid = PIDController(0.5, 0.0, 0.0, 0.0, 26.18);
};

#endif // MODULE_H