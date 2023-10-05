// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>
#include <list>
#include <iostream>

#include "Arm.h"
#include "Roller.h"
#include "autoBalence.h"
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>

#ifndef CFG_NO_DRIVEBASE

#include "swerve/Drivetrain.h"
#include "swerve/Odometry.h"
#include "swerve/SwerveModule.h"
#include "swerve/Trajectory.h"
#include "swerve/Vision.h"
#include "swerve/ngr.h"

#endif

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
  frc::Timer m_score_timer;
  void swerveDrive(bool const &field_relative);
  frc::SendableChooser<std::string> m_chooser;
  const std::string kScoreGoBack = "Score Go Back";
  const std::string kScoreDoNothing = "Score do nothing";
  const std::string kScoreBalance = "Score Balance";
  const std::string kScoreCrossLineBalance = "Score cross line balance";
// Score balance 
// Score Cross line balance
std::list <CONSTANTS::AUTO_ACTIONS> score_balance {
   CONSTANTS::AUTO_ACTIONS::SCORE,
   CONSTANTS::AUTO_ACTIONS::BALANCE,
  CONSTANTS::AUTO_ACTIONS::NOTHING,
};

  std::list<CONSTANTS::AUTO_ACTIONS> score_cross_line_bal {
    CONSTANTS::AUTO_ACTIONS::SCORE,
    CONSTANTS::AUTO_ACTIONS::CENTER_CROSS_LINE, 
    CONSTANTS::AUTO_ACTIONS::BALANCE,
    CONSTANTS::AUTO_ACTIONS::NOTHING,
  };

std::list<CONSTANTS::AUTO_ACTIONS> score_do_nothing {
  CONSTANTS::AUTO_ACTIONS::SCORE,
  CONSTANTS::AUTO_ACTIONS::NOTHING,
};

  std::list<CONSTANTS::AUTO_ACTIONS> score_go_back {
    CONSTANTS::AUTO_ACTIONS::SCORE,
    CONSTANTS::AUTO_ACTIONS::CROSS_LINE,
    CONSTANTS::AUTO_ACTIONS::CROSS_LINE_P,
    CONSTANTS::AUTO_ACTIONS::NOTHING,
  };
  
  std::list<CONSTANTS::AUTO_ACTIONS> *m_auto_sequence;

  std::string m_autoSelected;
  CONSTANTS::AUTO_ACTIONS m_action = CONSTANTS::AUTO_ACTIONS::NOTHING;
  CONSTANTS::STATE m_state = CONSTANTS::STATE::STOWED;
  bool m_stowed_toggle = true;
  bool m_intake_toggle = false;
  bool m_extake_low_toggle = false;
  bool m_extake_mid_toggle = false;
  bool m_extake_high_toggle = false;
  bool m_man_intake_toggle = false;
   bool m_man_extake_toggle = false;

  pathplanner::PathPlannerTrajectory m_fallback_traj;
  Arm m_arm;
  autoBalance m_auto_balence;
  Roller m_roller{ m_state };
#ifndef CFG_NO_DRIVEBASE
  Drivetrain m_drivetrain;
  Odometry m_odometry{ &m_drivetrain };
  Vision m_vision{ &m_drivetrain, &m_odometry }; 
  Trajectory m_trajectory{ &m_drivetrain, &m_odometry};
#endif
};
