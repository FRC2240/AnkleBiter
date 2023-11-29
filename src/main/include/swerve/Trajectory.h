#pragma once

#include "swerve/Drivetrain.h"

#include <units/acceleration.h>
#include <units/angle.h>
#include <units/length.h>
#include <pathplanner/lib/path/PathPlannerTrajectory.h>
#include <pathplanner/lib/path/PathPoint.h>
#include <pathplanner/lib/auto/AutoBuilder.h>

#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include "Constants.h"
#include <functional>
#include <cmath>
#include "swerve/ngr.h"
#include "swerve/Odometry.h"
#include <frc/DriverStation.h>

#include <frc/controller/HolonomicDriveController.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <frc2/command/SubsystemBase.h>
#include <chrono>
#include <thread>
#include <frc/Timer.h>

#ifndef CFG_NO_DRIVEBASE
using namespace pathplanner; // PathPlanner keeps everything hidden behind 2 sets of namespaces so it's safe to remove the first layer

class Trajectory : public frc2::SubsystemBase
{
public:
    Trajectory(
        Drivetrain *drivetrain,
        Odometry *odometry);

    // Note: a 2023 comment means it is Moonwalker Specific and can be safely removed.

    // 2023
    struct TrajDepends
    {
        units::meter_t current_x;
        units::meter_t current_y;
        units::degree_t current_head;
        units::degree_t current_rot;
        units::meter_t desired_x;
        units::meter_t desired_y;
        units::degree_t desired_head;
        units::degree_t desired_rot;
    };
    const PathConstraints DEFAULT_CONSTRAINTS = PathConstraints(14_fps, 7_fps_sq, 360_deg_per_s, 720_deg_per_s_sq);
    // constexpr PathConstraints DEFAULT_CONSTRAINTS = PathConstraints(CONSTANTS:)

    frc2::CommandPtr make_absolute_line_path(frc::Pose2d target_pose);

    frc2::CommandPtr make_relative_line_path(units::meter_t x, units::meter_t y, frc::Rotation2d rot);

    frc2::CommandPtr extract();
    /*
        TrajDepends fall_back(units::meter_t fallback_pos = 1.0_m);

        PathPlannerPath generate_live_traj(TrajDepends t);

        PathPlannerTrajectory generate_live_traj(units::meter_t current_x,
                                                 units::meter_t current_y,
                                                 frc::Rotation2d current_head,
                                                 frc::Rotation2d current_rot,
                                                 units::meter_t desired_x,
                                                 units::meter_t desired_y,
                                                 frc::Rotation2d desired_head,
                                                 frc::Rotation2d desired_rot
                                                 );

        PathPlannerTrajectory generate_live_traj(units::meter_t current_x,
                                                 units::meter_t current_y,
                                                 units::degree_t current_head,
                                                 units::degree_t current_rot,
                                                 units::meter_t desired_x,
                                                 units::meter_t desired_y,
                                                 units::degree_t desired_head,
                                                 units::degree_t desired_rot
                                                 );

        /// @brief Must be called a cycle before a trajectory is followed. Should only be called once, not periodicly.
        /// @param traj The trajectory to be followed. Does not affect the trajectory.
        /// @param offset The time offset for when the trajectory should start.
        void init_live_traj(PathPlannerTrajectory traj, units::second_t offset = 0.0_s);

        /// @brief Makes the robot start follwing a trajectory. Must be called every cycle.
        /// @param traj The trajectory to follow
        /// @return True if the path is complete, false if it in progress.
        bool follow_live_traj(PathPlannerTrajectory traj);

        /// @brief Converts a file into a live trajectory
        PathPlannerPath extract(std::string const &traj_dir,
                                    units::meters_per_second_t const &max_vel = Drivetrain::TRAJ_MAX_SPEED,
                                    units::meters_per_second_squared_t const &max_accl = Drivetrain::TRAJ_MAX_ACCELERATION);

        void printRobotRelativeSpeeds();

        void printFieldRelativeSpeeds();

        void driveToState(PathPlannerTrajectory::State const &state);

        /// @brief The legacy way of following a path. Interupts the thread until the path is finished
        /// @param traj_dir The file to load the trajectory.
        /// Only looks in src/main/deploy/pathplanner and .path should be ommited


        void follow(std::string const &traj_dir,
                    std::function<void(units::second_t time)> const &periodic = nullptr,
                    units::meters_per_second_t const &max_vel = Drivetrain::TRAJ_MAX_SPEED,
                    units::meters_per_second_squared_t const &max_accl = Drivetrain::TRAJ_MAX_ACCELERATION);

        void testHolonomic(frc::Pose2d const &target_pose,
                           units::velocity::meters_per_second_t const &velocity,
                           frc::Rotation2d const &target_rot);


        bool reverse_trajectory = false;
    */
private:
    Drivetrain *m_drivetrain;
    Odometry *m_odometry;
};
#endif
