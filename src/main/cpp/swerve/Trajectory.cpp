#include "swerve/Trajectory.h"

#ifndef CFG_NO_DRIVEBASE
/******************************************************************/
/*                        Private Variables                       */
/******************************************************************/

// This is using lambdas in order to use setters at beginning of runtime & save performance later
/*static frc::HolonomicDriveController controller{
    frc2::PIDController{1, 0, 0},
    frc2::PIDController{1, 0, 0},*/
/*frc::ProfiledPIDController<units::radian>{
    //IMPORTANT: THIS DOES NOTHING.
    40, 0, 0,
    //0.8, 0.0, 0.0,
    frc::TrapezoidProfile<units::radian>::Constraints{

        CONSTANTS::DRIVE::TRAJ_MAX_ANGULAR_SPEED,
        CONSTANTS::DRIVE::TRAJ_MAX_ANGULAR_ACCELERATION}}};*/

frc::Timer m_trajTimer;

Trajectory::Trajectory(Drivetrain *drivetrain, Odometry *odometry)
    : m_drivetrain{drivetrain}, m_odometry{odometry}
{
    AutoBuilder::configureHolonomic(
        [this]() -> frc::Pose2d
        {
            return m_odometry->getPose();
        },
        [this](frc::Pose2d pose) -> void
        {
            m_odometry->resetPosition(pose, frc::Rotation2d(m_drivetrain->get_absolute_angle()));
        },
        [this]() -> frc::ChassisSpeeds
        {
            return m_drivetrain->getRobotRelativeSpeeds();
        },
        [this](frc::ChassisSpeeds speeds) -> void
        {
            return m_drivetrain->drive(speeds);
        },
        HolonomicPathFollowerConfig(
            PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
            PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
            4.5_mps,                     // Max module speed, in m/s
            0.4_m,                       // Drive base radius in meters. Distance from robot center to furthest module.
            ReplanningConfig()           // Default path replanning config. See the API for the options here),
            ),
        this);
}

frc2::CommandPtr Trajectory::make_relative_line_path(units::meter_t x, units::meter_t y, frc::Rotation2d rot)
{
    std::vector<frc::Pose2d> points{
        m_odometry->getPose(), // First point is always where you are
        frc::Pose2d(x, y, rot)};

    std::vector<frc::Translation2d>
        bezierPoints = PathPlannerPath::bezierFromPoses(points);
    auto path = std::make_shared<PathPlannerPath>(bezierPoints, DEFAULT_CONSTRAINTS, GoalEndState(0.0_mps, rot));

    return AutoBuilder::followPathWithEvents(path);
}

frc2::CommandPtr Trajectory::make_absolute_line_path(frc::Pose2d target_pose)
{
    return AutoBuilder::pathfindToPose(target_pose, DEFAULT_CONSTRAINTS, 0.0_mps, 0.0_m);
}

/*
Trajectory::TrajDepends Trajectory::fall_back(units::meter_t fallback_pos)
{
    frc::Pose2d current_pose = m_odometry->getPose();
    Trajectory::TrajDepends ret;

        ret.desired_x = current_pose.X() - fallback_pos;
        ret.desired_y = current_pose.Y();

    auto heading = (frc::Translation2d(ret.desired_x, ret.desired_y) - current_pose.Translation()).Angle().Degrees();
    ret.desired_head = heading;
    // ret.desired_rot = 180_deg;
    ret.desired_rot = 0_deg;
    ret.current_rot = current_pose.Rotation().Degrees();
    ret.current_head = heading;
    ret.current_x = current_pose.X();
    ret.current_y = current_pose.Y();

    ret.desired_x = 2.0*ret.current_x - ret.desired_x;
    std::cout << "desired x: " << ret.desired_x.value() << std::endl;
    return ret;
}
/******************************************************************/
/*                   Public Function Definitions                  */
/******************************************************************/

// void Trajectory::printRobotRelativeSpeeds()
// {

//     // frc::SmartDashboard::PutNumber("Estimated VX Speed", robot_relative.vx.value());
//     // frc::SmartDashboard::PutNumber("Estimated VY Speed", robot_relative.vy.value());
//     // frc::SmartDashboard::PutNumber("Estimated Omega Speed", units::degrees_per_second_t{robot_relative.omega}.value() / 720);
// }
/*
PathPlannerPath Trajectory::extract(std::string const &traj_dir,
                                units::meters_per_second_t const &max_vel,
                                units::meters_per_second_squared_t const &max_accl)
{
    return PathPlannerPath::fromPathFile(traj_dir, max_vel, max_accl, reverse_trajectory);
}
// */
// PathPlannerPath Trajectory::generate_live_traj(TrajDepends t)
// {
//     std::vector<frc::Translation2d> bez_curve = PathPlannerPath::bezierFromPoses(
//         std::vector<frc::Pose2d> {
//         frc::Pose2d(t.current_x, t.current_y, frc::Rotation2d(t.current_head)),
//         frc::Pose2d(t.desired_x , t.desired_y, frc::Rotation2d(t.desired_head)),
//     });
//     // return PathPlannerPath(bez_curve, Path)

//     /*
//     return
//         PathPlannerTrajectory::(

//                                   PathConstraints(m_drivetrain->TRAJ_MAX_SPEED/2.5,
//                                                   m_drivetrain->TRAJ_MAX_ACCELERATION/2.5),

//                                   PathPoint(frc::Translation2d(t.current_x,
//                                                                t.current_y),
//                                             frc::Rotation2d(t.current_head),
//                                             frc::Rotation2d(t.current_rot)
//                                             ),
//                                   PathPoint(frc::Translation2d(t.desired_x,
//                                                                 t.desired_y),
//                                             frc::Rotation2d(t.desired_head),
//                                             frc::Rotation2d(t.desired_rot)
//                                             )
//                                   );
//                                   */

// }

// PathPlannerTrajectory Trajectory::generate_live_traj(units::meter_t current_x,
//                                                      units::meter_t current_y,
//                                                      frc::Rotation2d current_head,
//                                                      frc::Rotation2d current_rot,
//                                                      units::meter_t desired_x,
//                                                      units::meter_t desired_y,
//                                                      frc::Rotation2d desired_head,
//                                                      frc::Rotation2d desired_rot
//                                                      )
// {
//     PathPlannerTrajectory ret_val =
//         PathPlanner::generatePath(
//                                   PathConstraints(m_drivetrain->TRAJ_MAX_SPEED/2,
//                                                   m_drivetrain->TRAJ_MAX_ACCELERATION/2),

//                                   PathPoint(frc::Translation2d(current_x,
//                                                                current_y),
//                                             current_head,
//                                             current_rot
//                                             ),

//                                   PathPoint(frc::Translation2d(desired_x,
//                                                                desired_y),
//                                             desired_head,
//                                             desired_rot
//                                             )
//                                   );
//     return ret_val;
// }

// void Trajectory::init_live_traj(PathPlannerTrajectory traj, units::second_t offset)
// {
//     auto inital_state = traj.getInitialState();
//     if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue)
//     {
//         inital_state = pathplanner::PathPlannerTrajectory::transformStateForAlliance(inital_state, frc::DriverStation::Alliance::kRed);
//     }
//     auto const inital_pose = inital_state.pose;

//     // It is necessary to take the frc::Pose2d object from the state, extract its X & Y components, and then take the holonomicRotation
//     // to construct a new Pose2d as the original Pose2d's Z (rotation) value uses non-holonomic math
//     m_odometry->resetPosition({inital_pose.Translation(), inital_state.holonomicRotation}, m_drivetrain->getCCWHeading());

//     m_trajTimer.Reset();
//     m_trajTimer.Start();

//     if constexpr (CONSTANTS::DEBUGGING)
//         {
//             // If needed, we can disable the "error correction" for x & y
//             controller.SetEnabled(true);

//             // frc::SmartDashboard::PutString("Inital State: ", fmt::format("X: {}, Y: {}, Z: {}, Holonomic: {}\n", inital_pose.X().value(), inital_pose.Y().value(), inital_pose.Rotation().Degrees().value(), inital_state.holonomicRotation.Degrees().value()));
//         }
// }

// bool Trajectory::follow_live_traj(PathPlannerTrajectory traj)
// {

//     if ( (m_trajTimer.Get() <= traj.getTotalTime() + 0.02_s))
//     {
//         auto current_time = m_trajTimer.Get();

//         auto sample = traj.sample(current_time);

//         if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue)
//         {
//             sample = pathplanner::PathPlannerTrajectory::transformStateForAlliance(sample, frc::DriverStation::Alliance::kRed);
//         }
//         //std::cout << sample.pose.X().value() << " " <<  sample.pose.Y().value() << " " << sample.holonomicRotation.Degrees().value() << "\n";

//         m_odometry->getField2dObject("Traj")->SetPose({sample.pose.X(), sample.pose.Y(), sample.holonomicRotation});

//         driveToState(sample);
//         m_odometry->update();

//         //if (periodic)
//         //    periodic(current_time);

//         if constexpr (CONSTANTS::DEBUGGING)
//         {

//             static int trajectory_samples{};
//             frc::SmartDashboard::PutString("Sample:", fmt::format(
//                                                                   "Current trajectory sample value: {}, Pose X: {}, Pose Y: {}, Pose Z: {}\nHolonomic Rotation: {}, Timer: {}\n",
//                                                                   ++trajectory_samples, sample.pose.X().value(), sample.pose.Y().value(), sample.pose.Rotation().Degrees().value(),
//                                                                   sample.holonomicRotation.Degrees().value(), m_trajTimer.Get().value()));

//             printRobotRelativeSpeeds();
//             printFieldRelativeSpeeds();

//         }
//         // This is the refresh rate of the HolonomicDriveController's PID controllers (can be tweaked if needed)
//     }
//     else
//         {
//             m_drivetrain->stop();
//             return true;
//         }
//     return false;
// }

// void Trajectory::printFieldRelativeSpeeds()
// {

//     // frc::SmartDashboard::PutNumber("Real VX Speed", real_speeds.vx.value());
//     // frc::SmartDashboard::PutNumber("Real VY Speed", real_speeds.vy.value());
//     // frc::SmartDashboard::PutNumber("Real Omega Speed", units::degrees_per_second_t{real_speeds.omega}.value() / 720);
// }

// void Trajectory::driveToState(PathPlannerTrajectory::PathPlannerState const &state)
// {
//     // Correction to help the robot follow trajectory (combination of original trajectory speeds & error correction)
//     frc::ChassisSpeeds const correction = controller.Calculate(m_odometry->getPose(), state.pose, state.velocity, state.holonomicRotation);
//     m_drivetrain->faceDirection(correction.vx, correction.vy, state.holonomicRotation.Degrees(), false, 4, m_drivetrain->TRAJ_MAX_ANGULAR_SPEED);

//     if constexpr (debugging)
//     {

//     }
// }

// void Trajectory::follow(std::string const &traj_dir,
//                         std::function<void(units::second_t time)> const &periodic,
//                         units::meters_per_second_t const &max_vel,
//                         units::meters_per_second_squared_t const &max_accl)
// {
//     auto traj = PathPlanner::loadPath(traj_dir, max_vel, max_accl, reverse_trajectory);

//     auto const inital_state = traj.getInitialState();
//     auto const inital_pose = inital_state.pose;

//     // It is necessary to take the frc::Pose2d object from the state, extract its X & Y components, and then take the holonomicRotation
//     // to construct a new Pose2d as the original Pose2d's Z (rotation) value uses non-holonomic math
//     m_odometry->resetPosition({inital_pose.Translation(), inital_state.holonomicRotation}, m_drivetrain->getCCWHeading());

//     frc::Timer trajTimer;
//     trajTimer.Start();

//     if constexpr (debugging)
//     {
//         // If needed, we can disable the "error correction" for x & y
//         controller.SetEnabled(true);

//         frc::SmartDashboard::PutString("Inital State: ", fmt::format("X: {}, Y: {}, Z: {}, Holonomic: {}\n", inital_pose.X().value(), inital_pose.Y().value(), inital_pose.Rotation().Degrees().value(), inital_state.holonomicRotation.Degrees().value()));
//     }

//     while (frc::DriverStation::IsAutonomousEnabled() && (trajTimer.Get() <= traj.getTotalTime() + 0.1_s))
//     {
//         auto current_time = trajTimer.Get();

//         auto sample = traj.sample(current_time);

//         m_odometry->getField2dObject("Traj")->SetPose({sample.pose.X(), sample.pose.Y(), sample.holonomicRotation});

//         driveToState(sample);
//         m_odometry->update();

//         if (periodic)
//             periodic(current_time);

//         if constexpr (debugging)
//         {
//             static int trajectory_samples{};
//             frc::SmartDashboard::PutString("Sample:", fmt::format(
//                                                           "Current trajectory sample value: {}, Pose X: {}, Pose Y: {}, Pose Z: {}\nHolonomic Rotation: {}, Timer: {}\n",
//                                                           ++trajectory_samples, sample.pose.X().value(), sample.pose.Y().value(), sample.pose.Rotation().Degrees().value(),
//                                                           sample.holonomicRotation.Degrees().value(), trajTimer.Get().value()));
//             printRobotRelativeSpeeds();
//             printFieldRelativeSpeeds();
//         }

//         using namespace std::chrono_literals;
//         // This is the refresh rate of the HolonomicDriveController's PID controllers (can be tweaked if needed)
//         std::this_thread::sleep_for(20ms);
//     }
//     m_drivetrain->stop();
// }

// void Trajectory::testHolonomic(frc::Pose2d const &target_pose, units::velocity::meters_per_second_t const &velocity, frc::Rotation2d const &target_rot)
// {
//     m_drivetrain->drive(controller.Calculate(m_odometry->getPose(), target_pose, velocity, target_rot));
// }
// */

#endif
