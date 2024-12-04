#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include <vector>
#include <chrono>
#include <Alfredo_NoU3.h>
#include <SimpleFOC.h>
#include <SimpleDCMotor.h>
#include <SimpleFOCDrivers.h>

enum AngleUnit {
    RADIANS,
    DEGREES
};

struct Angle {
    float angle;
    AngleUnit unit = RADIANS;

    Angle(float angle, AngleUnit unit) : angle(angle), unit(unit) {
        if (unit == DEGREES) {
            this->angle = radians(angle);
            this->unit = RADIANS;
        }
    }

    Angle (float x, float y) {
        this->angle = atan2(y, x);
        this->unit = RADIANS;
    }

    Angle(float angle) : angle(angle) {}

    float getRadians() {
        return angle;
    }

    float getDegrees() {
        return degrees(angle);
    }

    Angle wrap() {
        float angleDegrees = this->unit == DEGREES ? angle : degrees(angle); 
        float wrapped = fmod(angleDegrees, 360.0f);
        if (wrapped < 0) {
            wrapped += 360.0f;
        }
        else if (wrapped >= 360.0f) {
            wrapped -= 360.0f;
        }
        return Angle(wrapped, DEGREES);
    }

    Angle wrapNeg180To180() {
        float angleDegrees = this->unit == DEGREES ? angle : degrees(angle); 
        if (angleDegrees > 180.0f) {
            angleDegrees -= 360.0f;
        }
        else if (angleDegrees < -180.0f) {
            angleDegrees += 360.0f;
        }
        return Angle(angleDegrees, DEGREES);
    }

    Angle rotateBy(float angleRotate, AngleUnit unit) {
        if (unit == DEGREES) {
            angleRotate = radians(angleRotate);
        }
        return Angle(this->angle + angleRotate);
    }

    Angle operator+(Angle other) {
        return Angle(angle + other.getRadians());
    }

    Angle operator-(Angle other) {
        return Angle(angle - other.getRadians());
    }

    Angle operator*(float scalar) {
        return Angle(angle * scalar);
    }

    Angle operator/(float scalar) {
        return Angle(angle / scalar);
    }

    bool operator==(Angle other) {
        return angle == other.getRadians();
    }

    bool operator!=(Angle other) {
        return angle != other.getRadians();
    }

    bool operator<(Angle other) {
        return angle < other.getRadians();
    }

    bool operator>(Angle other) {
        return angle > other.getRadians();
    }

    bool operator<=(Angle other) {
        return angle <= other.getRadians();
    }

    bool operator>=(Angle other) {
        return angle >= other.getRadians();
    }

    Angle operator+=(Angle other) {
        angle += other.getRadians();
        return *this;
    }

    Angle operator-=(Angle other) {
        angle -= other.getRadians();
        return *this;
    }

    Angle operator*=(float scalar) {
        angle *= scalar;
        return *this;
    }

    Angle operator/=(float scalar) {
        angle /= scalar;
        return *this;
    }

    Angle operator%(float scalar) {
        return Angle(fmod(angle, scalar));
    }

};

class Motor {
    public:
        Motor(int enc1, int enc2, NoU_Motor* rawMotor, Encoder* encoder);
        void setVelocity(Angle velocity);
        void stop();
        Angle getVelocity();
        Angle getPosition();
        void setInverted(bool inverted);
        void begin();
        void loop();

        bool inverted = false;

        int enc1;
        int enc2;

        NoU_Motor* rawMotor;
        Encoder* sensor;

        Angle MAX_SPEED = Angle(10*M_PI);

        long lastUpdate = 0;
    private:
};

#endif // MOTOR_H