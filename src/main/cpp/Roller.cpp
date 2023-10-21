#include "Roller.h"

Roller::Roller(CONSTANTS::STATE &state)
: m_state {state}
{
    m_right_roller_motor.SetSmartCurrentLimit(40);
    m_left_roller_motor.SetSmartCurrentLimit(40);
    m_right_roller_motor.Follow(m_left_roller_motor, true);

}

Roller::~Roller()
{
}

void Roller::spin(double speed)
{
    m_left_roller_motor.Set(speed);
}

/*bool Roller::is_loaded()
{
    //frc::SmartDashboard::PutNumber("range", m_tof_sensor.GetRange());
    return (
        //(units::millimeter_t {m_tof_sensor.GetRange()} < CONSTANTS::ROLLER::LOADED_RANGE)
    );
}*/
