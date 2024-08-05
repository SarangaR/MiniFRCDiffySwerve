#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

#include <Arduino.h>
#include <Module.h>
#include <vector>

class Drivetrain {
    public:
        Drivetrain(Module left, Module right);
        void drive(double vx, double vy, double omega);
        std::vector<moduleState> toSwerveModuleStates();
        void stop();
        void begin();

    private:
        Module left;
        Module right;

};

#endif // DRIVETRAIN_H