#include "Roller.h"

Roller::Roller(/* args */)
{
    m_right_roller_motor.Follow(m_left_roller_motor);

}

Roller::~Roller()
{
}

void Roller::spin(double speed)
{
    m_left_roller_motor.Set(speed);
}

bool Roller::is_loaded()
{
    return (m_left_roller_motor.GetOutputCurrent() >
            CONSTANTS::ARM::LOADED_CURRENT);
}
