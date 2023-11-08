#include <rev/CANSparkMax.h>
#include "Constants.h"
#include <TimeOfFlight.h>
#include <units/length.h>
#include <frc/smartdashboard/SmartDashboard.h>

//void spin(double speed);

//  rev::CANSparkMax m_left_roller_motor {CONSTANTS::ROLLER::LEFT_MOTOR_ID, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
  //rev::CANSparkMax m_right_roller_motor {CONSTANTS::ROLLER::RIGHT_MOTOR_ID, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
//  rev::SparkMaxPIDController m_roller_pid = m_left_roller_motor.GetPIDController();
//  rev::SparkMaxRelativeEncoder m_roller_encoder = m_left_roller_motor.GetEncoder();
//  CONSTANTS::PidCoeff m_roller_coeff = {1};
class Roller
{
    public:  
        Roller(CONSTANTS::STATE &state);
        ~Roller();
        void spin(double speed);
        bool is_loaded();



    private:
    CONSTANTS::STATE m_state;
    frc::TimeOfFlight m_tof_sensor {CONSTANTS::ROLLER::TOF_CAN};

    rev::CANSparkMax m_left_roller_motor {CONSTANTS::ROLLER::LEFT_MOTOR_ID, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
    rev::CANSparkMax m_right_roller_motor {CONSTANTS::ROLLER::RIGHT_MOTOR_ID, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
    
    rev::SparkMaxPIDController m_roller_pid = m_left_roller_motor.GetPIDController();
    
    rev::SparkMaxRelativeEncoder m_roller_encoder = m_left_roller_motor.GetEncoder();

   
};