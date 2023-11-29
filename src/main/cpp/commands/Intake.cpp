//This file is needlessy verbose, but is shows how to use a custom command class. 
//See Arm::stow() for a consise version, probably what your's should look like.


#include "commands/Intake.h"

Intake::Intake() = default;

Intake::Intake(Arm* arm)
: m_arm{arm}
{
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(m_arm);
}

// Called when the command is initially scheduled.
void Intake::Initialize() {
  frc::SmartDashboard::PutString("state", "intake");
}

// Called repeatedly when this Command is scheduled to run
void Intake::Execute() {
  m_arm->move(CONSTANTS::ARM::INTAKE_POS);
  m_arm->spin(CONSTANTS::ARM::INTAKE_VEL);
  frc::SmartDashboard::PutBoolean("loaded", m_arm->is_loaded());

}

// Called once the command ends or is interrupted.
void Intake::End(bool interrupted) {
  frc::SmartDashboard::PutString("state", "null");
}

// Returns true when the command should end.
bool Intake::IsFinished() {
  return m_arm->is_loaded();
}
