#ifndef ROBOTPOSE_H
#define ROBOTPOSE_H

#include <Arduino.h>
#include <SparkFun_Qwiic_OTOS_Arduino_Library.h>
#include <Filters/Butterworth.hpp>
#include <Filters/SMA.hpp>
#include <Kalman.h>
#include <Motor.h>

class RobotPose {
    public:
        RobotPose();
        bool begin(float magnetic_declination);
        void setYawOffset(float offset);
        void setYawOffset(float mag_x, float mag_y);
        void update(float mag_x, float mag_y);
        Angle getHeading();
        void calibrateHeading();
        void resetTracking();
        void setUnits(sfe_otos_linear_unit_t linearUnit, sfe_otos_angular_unit_t angularUnit);
        void setOTOSLinearScalar(float scalar);
        void setAngularScalar(float scalar);
        void setOTOSAngularScalar(float scalar);
        sfe_otos_pose2d_t getPosition();

    private:
        QwiicOTOS otos;
        sfe_otos_pose2d_t robotPose;
        float yaw_offset = 0;
        float mag_dec = 0;
        const double f_s = 100;
        const double f_c = 25;
        const double f_n = 2 * f_c / f_s;
        SOSFilter<float, 3U, BiQuadFilterDF1<float>> xFilter = butter<6>(f_n);
        SOSFilter<float, 3U, BiQuadFilterDF1<float>> yFilter = butter<6>(f_n);
        SOSFilter<float, 3U, BiQuadFilterDF1<float>> hFilter = butter<6>(f_n);
        float lastHeadingUpdate = 0;
        float currentYaw = 0;
        float angularScalar = 1;

        float mag_weight = 0.9;
        float sensor_imu_weight = 0.1;

        void updateHeading(float mag_x, float mag_y);
};
#endif // ROBOTPOSE_H