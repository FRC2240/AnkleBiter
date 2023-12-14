// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc2/command/CommandScheduler.h>

void Robot::RobotInit() {}

/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want to run during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic()
{
  frc2::CommandScheduler::GetInstance().Run();
}

/**
 * This function is called once each time the robot enters Disabled mode. You
 * can use it to reset any subsystem information you want to clear when the
 * robot is disabled.
 */
void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

/**
 * This autonomous runs the autonomous command selected by your {@link
 * RobotContainer} class.
 */
void Robot::AutonomousInit()
{
  m_autonomousCommand = m_container.GetAutonomousCommand();

  if (m_autonomousCommand)
  {
    m_autonomousCommand->Schedule();
  }
}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit()
{
  m_container.m_odometry.putField2d();
  // This makes sure that the autonomous stops running when
  // teleop starts running. If you want the autonomous to
  // continue until interrupted by another command, remove
  // this line or comment it out.
  if (m_autonomousCommand)
  {
    m_autonomousCommand->Cancel();
  }
}

void Robot::swerveDrive(bool const &field_relative)
{
  m_container.m_drivetrain.print_angle();

  if (m_container.m_driverController.GetStartButtonReleased())
  {
    m_container.m_drivetrain.zero_yaw();
  }

  frc::SmartDashboard::PutNumber("navx", m_container.m_drivetrain.getAngle().value());

  const units::meters_per_second_t left_right{-frc::ApplyDeadband(m_container.m_driverController.GetLeftX(), 0.1) * CONSTANTS::DRIVE::TELEOP_MAX_SPEED};
  frc::SmartDashboard::PutNumber("desired lr translation", left_right.value());
  const units::meters_per_second_t front_back{frc::ApplyDeadband(m_container.m_driverController.GetLeftY(), 0.1) * CONSTANTS::DRIVE::TELEOP_MAX_SPEED};
  frc::SmartDashboard::PutNumber("desired fb translation", front_back.value());
  auto const rot = frc::ApplyDeadband(m_container.m_driverController.GetRightX(), .1) * m_container.m_drivetrain.TELEOP_MAX_ANGULAR_SPEED;
  frc::SmartDashboard::PutNumber("desired rotation", rot.value());
  m_container.m_drivetrain.drive(front_back, -left_right, -rot, field_relative);

  // frc::SmartDashboard::PutNumber("Gyro: ", m_drivetrain.getAngle().value());
  // frc::SmartDashboard::PutNumber("front/back: ", front_back.value());
  // frc::SmartDashboard::PutNumber("left/right: ", left_right.value());
}

/**
 * This function is called periodically during operator control.
 */
void Robot::TeleopPeriodic()
{
  m_container.m_odometry.update();
}

/**
 * This function is called periodically during test mode.
 */
void Robot::TestPeriodic() {}

/**
 * This function is called once when the robot is first started up.
 */
void Robot::SimulationInit() {}

/**
 * This function is called periodically whilst in simulation.
 */
void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
