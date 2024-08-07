#ifndef MODULE_H
#define MODULE_H

#include <Motor.h>

struct moduleState {
    double speed;
    double angle;

    moduleState(double speed, double angle) :
        speed(speed),
        angle(angle)
    {}
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
        std::vector<double> rotateModule(double angle);
        double getMotorSpeedsForAngle(double angleDegrees);
        double getMotorSpeedsForSpeed(double speed);
        moduleState optimize(moduleState state);
        moduleState getState();
        double rotateAngleBy(double angle, double angleToRotateBy);
        double wrap0To360(double angle);
        double wrapNeg180To180(double angle);
        void resetState();
        double getError(double degrees);

        moduleID id;
    private:
        Motor* top;
        Motor* bottom;

        double prevErrorAngle = 0;
};

#endif // MODULE_H