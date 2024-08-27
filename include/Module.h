#ifndef MODULE_H
#define MODULE_H

#include <Motor.h>


/**
 * @brief Represents the state of a module with speed and angle.
 */
struct moduleState {
    double speed; ///< Speed of the module
    double angle; ///< Angle of the module

    /**
     * @brief Constructs a new moduleState object.
     * 
     * @param speed The speed of the module.
     * @param angle The angle of the module.
     */
    moduleState(double speed, double angle) :
        speed(speed),
        angle(angle)
    {}

    /**
     * @brief Converts the module state to a string representation.
     * 
     * @return A string representing the speed and angle of the module.
     */
    String toString() {
        return "Speed: " + String(speed) + " Angle: " + String(angle);
    }
};

/**
 * @brief Enum for module identifiers.
 */
enum moduleID {
    LEFT, ///< Left module
    RIGHT ///< Right module
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

        void setInverted(bool inverted);

        void loop();

        moduleID id;
    private:
        Motor* top;
        Motor* bottom;

        double prevErrorAngle = 0;

        double wheelRadius = 0.05;
        double gearRatio = 0.4;

        float profilePos = 0;
        double angleTarget = 0;
        double speedTarget = 0;

        bool alreadyDone = false;

        PIDController pid = PIDController();
};

#endif // MODULE_H