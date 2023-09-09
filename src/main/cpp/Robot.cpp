// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include "Buttons.h"
#include <fmt/core.h>
#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
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
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  fmt::print("Auto selected: {}\n", m_autoSelected);

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {
  if (BUTTON::STOWED())
  {
    m_stowed_toggle = !m_stowed_toggle;
    m_intake_toggle = 0;
    m_man_intake_toggle = 0;
    m_extake_toggle = 0;
  }
   if (BUTTON::INTAKE())
  {
    m_stowed_toggle = 0;
    m_intake_toggle = !m_intake_toggle;
    m_man_intake_toggle = 0;
    m_extake_toggle = 0;
  }
   if (BUTTON::MAN_INTAKE())
  {
    m_stowed_toggle = 0;
    m_intake_toggle = 0;
    m_man_intake_toggle = !m_man_intake_toggle;
    m_extake_toggle = 0;
  }
   if (BUTTON::EXTAKE())
  {
    m_stowed_toggle = 0;
    m_intake_toggle = 0;
    m_man_intake_toggle = 0;
    m_extake_toggle = !m_extake_toggle;
  }
  switch (m_state){ 
    case CONSTANTS::STATE::STOWED:
    m_arm.move(CONSTANTS::ARM::STORE_POS);
    
    m_roller.spin(CONSTANTS::ROLLER::SLOW);
   
    // 1. Return arm to intake point
    // 2. Remain in intake point
    // 3. Spin rollers slowly to keep cube in
    break;
    case CONSTANTS::STATE::MAN_INTAKE: //NOTE: No specific code needs to be written for man intake. If the state is man intake, is_loaded will always return false.
    //Fallthough inferred 
    case CONSTANTS::STATE::INTAKE:
    /*Make sure arm is in correct postition, use Arm Move to put it in correct postions, then spin Motors, spin until it says loaded, change state to STOW*/
    m_arm.move(CONSTANTS::ARM::INTAKE_POS);
    
    m_roller.spin(1);
    
    if (m_roller.is_loaded())
    {
      m_state = CONSTANTS::STATE::STOWED;
    }
    break;
    case CONSTANTS::STATE::EXTAKE:
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
int main() {
  return frc::StartRobot<Robot>();
}
#endif
