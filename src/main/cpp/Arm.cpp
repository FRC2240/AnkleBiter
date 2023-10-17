#include "Arm.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include <iostream>

Arm::Arm() {
    m_left_arm_motor.SetSmartCurrentLimit(50);
    m_right_arm_motor.SetSmartCurrentLimit(50);

    m_arm_pid.SetP(m_arm_coeff.p);
    m_arm_pid.SetD(m_arm_coeff.d);
    m_arm_pid.SetOutputRange(m_arm_coeff.min, m_arm_coeff.max);
    std::cout << "P: " << m_arm_coeff.p << "\n";
   m_right_arm_motor.Follow(m_left_arm_motor, true);
   m_arm_encoder.SetZeroOffset(0.547);
       m_arm_pid.SetFeedbackDevice(m_arm_encoder);
    

}

bool Arm::is_good()
{
    return !(
        m_left_arm_motor.GetFault(rev::CANSparkMax::FaultID::kSensorFault)  ||
        m_right_arm_motor.GetFault(rev::CANSparkMax::FaultID::kSensorFault) ||
        m_left_arm_motor.GetFault(rev::CANSparkMax::FaultID::kMotorFault)   ||
        m_right_arm_motor.GetFault(rev::CANSparkMax::FaultID::kMotorFault)  ||
        m_right_arm_motor.GetFault(rev::CANSparkMax::FaultID::kDRVFault)    ||
        m_left_arm_motor.GetFault(rev::CANSparkMax::FaultID::kDRVFault)     ||
        m_right_arm_motor.GetFault(rev::CANSparkMax::FaultID::kOtherFault)  ||
        m_left_arm_motor.GetFault(rev::CANSparkMax::FaultID::kOtherFault)   ||
        m_right_arm_motor.GetFault(rev::CANSparkMax::FaultID::kCANRX)       ||
        m_left_arm_motor.GetFault(rev::CANSparkMax::FaultID::kCANRX)        ||
        m_right_arm_motor.GetFault(rev::CANSparkMax::FaultID::kCANTX)       ||
        m_left_arm_motor.GetFault(rev::CANSparkMax::FaultID::kCANTX)
    );
}

void Arm::move(double setpoint)
{
    // m_left_arm_motor.Set(0.1);
    m_arm_pid.SetReference(setpoint,
                    rev::CANSparkMax::ControlType::kPosition);
        // std::cout << "here\n";
        frc::SmartDashboard::PutNumber("Pgain", m_arm_pid.GetP());
        frc::SmartDashboard::PutNumber("pos", m_arm_encoder.GetPosition());
        frc::SmartDashboard::PutNumber("vel", m_arm_encoder.GetVelocity());

        // std::cout << "P: " << m_arm_pid.GetP();

        frc::SmartDashboard::PutNumber("setpoint", setpoint);

}

double Arm::get_position()
{
    return m_arm_encoder.GetPosition();
}



void Arm::SimulationPeriodic() {}
