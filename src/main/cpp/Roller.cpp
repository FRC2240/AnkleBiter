#include "Roller.h"

Roller::Roller(CONSTANTS::STATE &state)
: m_state {state}
{
    m_right_roller_motor.Follow(m_left_roller_motor, true);

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
    return (
        m_state != CONSTANTS::STATE::MAN_INTAKE &&
        m_roller_encoder.GetVelocity() >
            CONSTANTS::ARM::LOADED_RPM
    );
}
