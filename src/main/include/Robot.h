// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>

#include "Arm.h"
#include "Roller.h"
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#ifndef CFG_NO_DRIVEBASE

#include "swerve/Drivetrain.h"
#include "swerve/Odometry.h"
#include "swerve/SwerveModule.h"
#include "swerve/Trajectory.h"
#include "swerve/Vision.h"
#include "swerve/ngr.h"
#endif CFG_NO_DRIVEBASE

class Robot : public frc::TimedRobot
{
public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;
  void SimulationInit() override;
  void SimulationPeriodic() override;

private:
  void swerveDrive(bool const &field_relative);
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;
  CONSTANTS::AUTO_ACTIONS m_action = CONSTANTS::AUTO_ACTIONS::NOTHING;
  CONSTANTS::STATE m_state = CONSTANTS::STATE::STOWED;
  bool m_stowed_toggle = true;
  bool m_intake_toggle = false;
  bool m_extake_toggle = false;
  bool m_man_intake_toggle = false;
  Arm m_arm;
  Roller m_roller{ m_state };
#ifndef CFG_NO_DRIVEBASE
  Drivetrain m_drivetrain;
  Odometry m_odometry{ &m_drivetrain };
  Vision m_vision{ &m_drivetrain, &m_odometry };
#endif
};
