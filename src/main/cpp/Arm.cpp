#include "Arm.h"
#include "frc/smartdashboard/SmartDashboard.h"

Arm::Arm() {
    m_arm_pid.SetP(m_arm_coeff.p);
    m_arm_pid.SetI(m_arm_coeff.i);

    m_roller_pid.SetP(m_arm_coeff.p);
    

    m_right_arm_motor.Follow(m_left_arm_motor);
    m_right_roller_motor.Follow(m_left_roller_motor);

}

void Arm::move(double setpoint)
{
    m_arm_pid.SetReference(setpoint,
                    rev::CANSparkMax::ControlType::kPosition);

        frc::SmartDashboard::PutNumber("setpoint", setpoint);

}

void Arm::spin(double speed)
{
    m_left_roller_motor.Set(speed);
    frc::SmartDashboard::PutNumber("speed", speed);
}

double Arm::get_position()
{
    return m_arm_encoder.GetPosition();
}

bool Arm::is_loaded()
{
    return (m_left_roller_motor.GetOutputCurrent() >
            CONSTANTS::ARM::LOADED_CURRENT);
}

void Arm::SimulationPeriodic() {}
