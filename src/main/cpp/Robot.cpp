// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include "Buttons.h"
#include <fmt/core.h>
#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit()
{
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
#ifndef CFG_NO_DRIVEBASE
  m_drivetrain.init();
  m_odometry.putField2d();
#endif
}

/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW
 * Dashboard, remove all of the chooser code and uncomment the GetString line
 * to get the auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the
 * SendableChooser make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit()
{
#ifndef CFG_NO_DRIVEBASE
  m_odometry.update();
  m_drivetrain.flip();
#endif
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  fmt::print("Auto selected: {}\n", m_autoSelected);

  if(m_autoSelected == kAutoNameCustom)
    {
      // Custom Auto goes here
    }
  else
    {
      // Default Auto goes here
    }
}

void Robot::AutonomousPeriodic()
{
  /*
  if(m_autoSelected == "Score Go Back")
    {
      m_auto_sequence = score_go_back;
    }
  else if(m_autoSelected == "Score do nothing")
    {
      m_auto_sequence = score_do_nothing;
    }
  else if(m_autoSelected == "Score Balance")
    {
      m_auto_sequence = score_balance;
    }
  else if(m_autoSelected == "Score cross line balance")
    {
      m_auto_sequence = score_cross_line_bal;
    }
  m_action = m_auto_sequence.front();
  switch(m_action)
    {
    case CONSTANTS::AUTO_ACTIONS::NOTHING:

      break;

    case CONSTANTS::AUTO_ACTIONS::BALANCE:
      m_auto_balence.auto_balance_routine();
      break;

    case CONSTANTS::AUTO_ACTIONS::CENTER_CROSS_LINE:
      m_fallback_traj = m_trajectory.generate_live_traj(
          m_trajectory.fall_back(CONSTANTS::TRAJECTORY::fall_back_center));
      m_trajectory.init_live_traj(m_fallback_traj);
      m_action = CONSTANTS::AUTO_ACTIONS::CROSS_LINE_P;
      break;

    case CONSTANTS::AUTO_ACTIONS::CROSS_LINE:
      m_fallback_traj = m_trajectory.generate_live_traj(
          m_trajectory.fall_back(CONSTANTS::TRAJECTORY::fall_back_dist));
      m_trajectory.init_live_traj(m_fallback_traj);
      m_action = CONSTANTS::AUTO_ACTIONS::CROSS_LINE_P;
      break;

    case CONSTANTS::AUTO_ACTIONS::CROSS_LINE_P:
      if(m_trajectory.follow_live_traj(m_fallback_traj))
        {
          m_auto_sequence.pop_front();
        }

      break;

    case CONSTANTS::AUTO_ACTIONS::SCORE:
      m_score_timer.Start();
      if(m_score_timer.Get() <= 0.5_s)
        {
          m_roller.spin(1);
        }
      else
        {
          m_auto_sequence.pop_front();
        }
      break;

    default:
      break;
    }
    */
}

void Robot::TeleopInit()
{
#ifndef CFG_NO_DRIVEBASE
  m_odometry.update();
#endif
}

void Robot::swerveDrive(bool const &field_relative)
{
#ifndef CFG_NO_DRIVEBASE
  const units::meters_per_second_t left_right{ -(frc::ApplyDeadband(
      BUTTON::DRIVETRAIN::LX(), CONSTANTS::DEADBAND)) * CONSTANTS::DRIVE::TELEOP_MAX_SPEED };

  const units::meters_per_second_t front_back{ -(frc::ApplyDeadband(
      BUTTON::DRIVETRAIN::LY(), CONSTANTS::DEADBAND)) * CONSTANTS::DRIVE::TELEOP_MAX_SPEED };
  auto const rot
      = frc::ApplyDeadband(BUTTON::DRIVETRAIN::RX(), CONSTANTS::DEADBAND)
        * (CONSTANTS::DRIVE::TELEOP_MAX_ANGULAR_SPEED);

  m_drivetrain.drive(front_back, -left_right, rot, field_relative);

  // frc::SmartDashboard::PutNumber("Gyro: ", m_drivetrain.getAngle().value());
  // frc::SmartDashboard::PutNumber("front/back: ", front_back.value());
  // frc::SmartDashboard::PutNumber("left/right: ", left_right.value());
#endif
}

void Robot::TeleopPeriodic()
{
  
  Robot::swerveDrive(true);

  if(BUTTON::STOWED())
    {
      m_stowed_toggle = !m_stowed_toggle;
      m_intake_toggle = 0;
      m_man_intake_toggle = 0;
      m_extake_toggle = 0;
    }
  if(BUTTON::INTAKE())
    {
      m_stowed_toggle = 0;
      m_intake_toggle = !m_intake_toggle;
      m_man_intake_toggle = 0;
      m_extake_toggle = 0;
    }
  if(BUTTON::MAN_INTAKE())
    {
      m_stowed_toggle = 0;
      m_intake_toggle = 0;
      m_man_intake_toggle = !m_man_intake_toggle;
      m_extake_toggle = 0;
    }
  if(BUTTON::EXTAKE())
    {
      m_stowed_toggle = 0;
      m_intake_toggle = 0;
      m_man_intake_toggle = 0;
      m_extake_toggle = !m_extake_toggle;
      m_score_timer.Reset();
    }

  if(m_extake_toggle)
    {
      m_state = CONSTANTS::STATE::EXTAKE;
    }
  if(m_intake_toggle)
    {
      m_state = CONSTANTS::STATE::INTAKE;
    }
  if(m_stowed_toggle)
    {
      m_state = CONSTANTS::STATE::STOWED;
    }
  if(m_man_intake_toggle)
    {
      m_state = CONSTANTS::STATE::MAN_INTAKE;
    }
  switch(m_state)
    {
    case CONSTANTS::STATE::STOWED:
      m_arm.move(CONSTANTS::ARM::STORE_POS);
      m_roller.spin(CONSTANTS::ROLLER::SLOW);
      m_roller.spin(CONSTANTS::FF_SPEED);
    frc::SmartDashboard::PutString("state", "stowed");

      break;
    case CONSTANTS::STATE::MAN_INTAKE: // NOTE: No specific code needs to be
                                       // written for man intake. If the state
                                       // is man intake, is_loaded will always
                                       // return false.
                                       // Fallthrough (lack of break) intended.
    case CONSTANTS::STATE::INTAKE:
    frc::SmartDashboard::PutString("state", "intake");
      m_arm.move(CONSTANTS::ARM::INTAKE_POS);
      m_roller.spin(CONSTANTS::ARM::INTAKE_VEL); // No idea if this is the right value/sign or not
  frc::SmartDashboard::PutBoolean("loaded", m_roller.is_loaded());
      if(m_roller.is_loaded())
        {
          m_intake_toggle = 0;
          m_stowed_toggle = 1;
          m_state = CONSTANTS::STATE::STOWED;
        }
      break;

    case CONSTANTS::STATE::EXTAKE:
      m_arm.move(CONSTANTS::ARM::SCORE_POS);
      m_roller.spin(-1);
      m_score_timer.Start();
      
        if(m_score_timer.Get() > units::time::second_t(0.5)) 
        {
          m_score_timer.Stop();
          m_score_timer.Reset();
          m_state = CONSTANTS::STATE::STOWED;
          m_extake_toggle = !m_extake_toggle;
        }
      break;
    }

}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
