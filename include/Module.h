#ifndef MODULE_H
#define MODULE_H

#include <Motor.h>


/**
 * @brief Represents the state of a module with speed and angle.
 */
struct moduleState {
    float speed; ///< Speed of the module
    float angle; ///< Angle of the module

    /**
     * @brief Constructs a new moduleState object.
     * 
     * @param speed The speed of the module.
     * @param angle The angle of the module.
     */
    moduleState(float speed, float angle) :
        speed(speed),
        angle(angle)
    {}

    float roundPlaces(float value, int places) {
        float factor = pow(10, places);
        return round(value * factor) / factor;
    }

    moduleState optimize(moduleState desiredState, Angle currentAngle) {
        auto delta = Angle(desiredState.angle, DEGREES) - currentAngle;
        if (fabs(delta.wrapNeg180To180().getDegrees()) > 90.0f) {
            return moduleState(-desiredState.speed, Angle(desiredState.angle, DEGREES).rotateBy(180, DEGREES).wrapNeg180To180().getDegrees());
        }
        else {
            return moduleState(desiredState.speed, desiredState.angle);
        }
    }

    void optimize(Angle currentAngle) {
        auto delta = Angle(angle, DEGREES) - currentAngle;
        if (fabs(delta.wrapNeg180To180().getDegrees()) > 90.0f) {
            speed = -speed;
            angle = Angle(angle, DEGREES).rotateBy(180, DEGREES).wrapNeg180To180().getDegrees();
        }
    }
    
    /**
     * @brief Converts the module state to a string representation.
     * 
     * @return A string representing the speed and angle of the module.
     */
    String toString() {
        return "Speed: " + String(roundPlaces(speed, 2)) + " Angle: " + String(roundPlaces(angle, 2));
    }
};

/**
 * @brief Enum for module identifiers.
 */
enum moduleID {
    LEFT, ///< Left module
    RIGHT, ///< Right module
    CENTER ///< Center module
};

class Module {
    public:
        Module(Motor* top, Motor* bottom, moduleID id);
        void setDesiredState(moduleState state);
        void stop();
        void begin();
        Angle getModuleOrientation();
        float getModuleSpeed();
        std::vector<float> rotateModule(float angle);
        float getMotorSpeedsForAngle(float angleDegrees);
        float getMotorSpeedsForSpeed(float speed);
        moduleState getState();
        float rotateAngleBy(float angle, float angleToRotateBy);
        float wrapNeg180To180(float angle);
        float getError(float degrees);
        float getProfileState();
        float getErrorModifier(float expected, float actual);

        void setInverted(bool inverted);
        void setMotorInvert(bool top, bool bottom);

        void loop();

        moduleID id;

        static constexpr float MAX_SPEED_SPIN_MS = 10.0f*M_PI * ((12.0f/30.0f)*(30.0f/10.0f)) * 0.05f;
    private:
        Motor* top;
        Motor* bottom;

        float prevErrorAngle = 0;

        float wheelRadius = 2.5f/100.0f;
        float gearRatioMotorToSun = 12.0f/30.0f;
        float gearRatioSunToWheelTurn = 1.0f/0.5f;
        float gearRatioTurn = gearRatioMotorToSun;
        float gearRatioSunToWheelSpin = 30.0f/10.0f;
        float gearRatioSpin = gearRatioMotorToSun * gearRatioSunToWheelSpin;

        float profilePos = 0;
        float angleTarget = 0;
        float speedTarget = 0;

        bool alreadyDone = false;

        float feedforwardGain = 4;

        PIDController pid = PIDController(5.0f, 0.0f, 0.05f , 0.0f, top->MAX_SPEED.getRadians());
};

#endif // MODULE_H