#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

#include <Arduino.h>
#include <Module.h>
#include <vector>

class Drivetrain {
    public:
        Drivetrain(Module* left, Module* right);
        void drive(double vx, double vy, double omega);
        std::vector<moduleState> toSwerveModuleStates(double vx, double vy, double omega);
        void stop();
        void begin();
        double getGyroAngle();
        void loop();
        moduleState optimize(moduleState desiredState, moduleState currentState);
        std::vector<double> normalizeSpeeds(std::vector<double> speeds);

    private:
        Module* left;
        Module* right;

        double LENGTH = 0.5;
        double WIDTH = 0.5;

};

#endif // DRIVETRAIN_H