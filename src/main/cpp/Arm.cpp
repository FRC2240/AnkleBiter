#include "Arm.h"
#include "frc/smartdashboard/SmartDashboard.h"

Arm::Arm() {
    m_arm_pid.SetP(m_arm_coeff.p);
    m_arm_pid.SetD(m_arm_coeff.d);

    

    m_right_arm_motor.Follow(m_left_arm_motor);

}

void Arm::move(double setpoint)
{
    m_arm_pid.SetReference(setpoint,
                    rev::CANSparkMax::ControlType::kPosition);

        frc::SmartDashboard::PutNumber("setpoint", setpoint);

}

double Arm::get_position()
{
    return m_arm_encoder.GetPosition();
}



void Arm::SimulationPeriodic() {}
