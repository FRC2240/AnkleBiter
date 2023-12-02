// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>

#include "commands/Autos.h"
#include "commands/ExampleCommand.h"

RobotContainer::RobotContainer()
{
  // Initialize all of your commands and subsystems here

  // Configure the button bindings
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings()
{
  // Configure your trigger bindings here

  // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
  frc2::Trigger([this]
                { return m_subsystem.ExampleCondition(); })
      .OnTrue(ExampleCommand(&m_subsystem).ToPtr());

  // Schedule `ExampleMethodCommand` when the Xbox controller's B button is
  // pressed, cancelling on release.
  // m_driverController.B().WhileTrue(m_subsystem.ExampleMethodCommand());

  m_driverController.RightTrigger().ToggleOnTrue(m_arm.move_low_command().AndThen(m_arm.extake_command(CONSTANTS::TARGET::LOW).WithTimeout(0.5_s)));
  // m_driverController.RightTrigger().OnTrue(m_arm.move_mid_command().AndThen(m_arm.extake_command(CONSTANTS::TARGET::MID).WithTimeout(0.5_s)));
  // m_driverController.RightBumper().OnTrue(m_arm.move_low_command().AndThen(m_arm.extake_command(CONSTANTS::TARGET::LOW).WithTimeout(0.5_s)));
  m_driverController.LeftBumper().ToggleOnTrue(Intake(&m_arm).ToPtr());
  m_arm.SetDefaultCommand(m_arm.stow_command());
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand()
{
  // An example command will be run in autonomous
  return autos::ExampleAuto(&m_subsystem);
}
