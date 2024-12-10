#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

#include <Arduino.h>
#include <Module.h>
#include <vector>
#include <ArduinoEigen.h>
#include <ArduinoEigenDense.h>
#include <PathPlanner.h>
#include <SparkFun_Qwiic_OTOS_Arduino_Library.h>

class Drivetrain {
    public:
        Drivetrain(Module* left, Module* right, Module* center);
        std::array<moduleState, 3> drive(float vx, float vy, float omega, Angle gyroAngle, bool fieldOriented);
        std::array<moduleState, 3> toSwerveModuleStates(float vx, float vy, float omega, Angle angle, bool fieldOriented);
        void stop();
        void begin();
        float getGyroAngle();
        void loop();
        void setBrake(bool brake);

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

        void followSpline(const std::vector<Point>& spline, sfe_otos_pose2d_t pose, float LOOKAHEAD_DISTANCE = 0.2, float KP_POSITION = 1.0, float KP_ORIENTATION = 1.0);

    private:
        Module* left;
        Module* right;
        Module* center;

        static const int numModules = 3;

        std::array<float, 3U> fromFieldRelativeSpeeds(float vx, float vy, float omega, Angle gyroAngle); 

        int findClosestPoint(const std::vector<Point>& spline, float x, float y);
        Point findLookaheadPoint(const std::vector<Point>& spline, int closestIndex, float lookaheadDistance, sfe_otos_pose2d_t pose);

        // Eigen::MatrixXd inverseKinematics = Eigen::MatrixXd(numModules*2, 3);
        // Eigen::MatrixXd forwardKinematics = Eigen::MatrixXd(3, numModules*2);

        // Eigen::Vector3d centerOfRotation = Eigen::Vector3d(0, 0, 0);
        // Eigen::Vector3d prevCoR = Eigen::Vector3d(0, 0, 0);

};

#endif // DRIVETRAIN_H