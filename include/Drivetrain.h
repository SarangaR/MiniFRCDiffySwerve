#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

#include <Arduino.h>
#include <Module.h>
#include <vector>
#include <ArduinoEigen.h>
#include <ArduinoEigenDense.h>

class Drivetrain {
    public:
        Drivetrain(Module* left, Module* right, Module* center);
        std::array<moduleState, 3> drive(float vx, float vy, float omega, Angle gyroAngle, bool fieldOriented, HWCDC *serial);
        std::array<moduleState, 3> toSwerveModuleStates(float vx, float vy, float omega, Angle angle, bool fieldOriented, HWCDC *serial);
        void stop();
        void begin();
        float getGyroAngle();
        void loop();
        std::vector<Angle> getModuleOrientations();
        std::vector<float> getModuleSpeeds();
        std::array<moduleState, 3> getModuleStates();
        std::array<moduleState, 3> optimize(std::array<moduleState, 3> desiredStates, std::array<moduleState, 3> currentStates);
        std::array<moduleState, 3> normalizeSpeeds(std::array<moduleState, 3> speeds);

        std::vector<String> lastModuleStates = {"", ""};

        //distance from wheel to wheel (trackwidth)
        float sideLength = 13.5f/100.0f; //m

        //distance from vertex to center of triangle (robot is triangle)
        float halfAngle = M_PI/6.0f;
        float centroidDistance = (0.5*sideLength)/(cos(halfAngle));
        float vertexYDistance = centroidDistance*sin(halfAngle);

        std::array<Eigen::Vector3d, 3> modulePositions = {
            Eigen::Vector3d(-0.5*sideLength, -vertexYDistance, 0),
            Eigen::Vector3d(0.5*sideLength, -vertexYDistance, 0),
            Eigen::Vector3d(0, 0.5*sideLength, 0)
        };

    private:
        Module* left;
        Module* right;
        Module* center;

        static const int numModules = 3;

        std::array<float, 3U> fromFieldRelativeSpeeds(float vx, float vy, float omega, Angle gyroAngle, HWCDC *serial); 

        // Eigen::MatrixXd inverseKinematics = Eigen::MatrixXd(numModules*2, 3);
        // Eigen::MatrixXd forwardKinematics = Eigen::MatrixXd(3, numModules*2);

        // Eigen::Vector3d centerOfRotation = Eigen::Vector3d(0, 0, 0);
        // Eigen::Vector3d prevCoR = Eigen::Vector3d(0, 0, 0);

};

#endif // DRIVETRAIN_H