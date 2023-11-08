#pragma once

#include "swerve/SwerveModule.h"
#include "swerve/Drivetrain.h"
#include "swerve/ngr.h"
#include "Constants.h"
#include <iostream>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/smartdashboard/SmartDashboard.h>


#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/geometry/Pose2d.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <TimeOfFlight.h>
#include <frc/DriverStation.h>

#ifndef CFG_NO_DRIVEBASE
class Odometry
{
    public:
        Odometry(Drivetrain* drivetrain);
        void putField2d();

    [[nodiscard]] frc::Pose2d getPose();

    void reset_position_from_vision(const frc::Pose2d &bot_pose);

    void reset_from_distance();

    void update();

    void resetPosition(const frc::Pose2d &pose,
                       const frc::Rotation2d &gyroAngle);

    [[nodiscard]] frc::FieldObject2d *getField2dObject(std::string_view name);

    [[nodiscard]] frc::ChassisSpeeds const getFieldRelativeSpeeds();
    private:
        Drivetrain* m_drivetrain;
};
#endif
