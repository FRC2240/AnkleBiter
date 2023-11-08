// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Arm.h"

Arm::Arm() {
    m_right_roller_motor.SetSmartCurrentLimit(40);
    m_left_roller_motor.SetSmartCurrentLimit(40);
    m_right_roller_motor.Follow(m_left_roller_motor, true);
    m_left_arm_motor.SetSmartCurrentLimit(50);
    m_right_arm_motor.SetSmartCurrentLimit(50);
    m_arm_pid.SetP(m_arm_coeff.p);
    m_arm_pid.SetD(m_arm_coeff.d);
    m_arm_pid.SetOutputRange(m_arm_coeff.min, m_arm_coeff.max);
    std::cout << "P: " << m_arm_coeff.p << "\n";
    m_right_arm_motor.Follow(m_left_arm_motor, true);
    m_arm_encoder.SetZeroOffset(0.547); //TODO: put this in constants
    m_arm_pid.SetFeedbackDevice(m_arm_encoder);
    
}

// This method will be called once per scheduler run
void Arm::Periodic() {
    //Nothing is needed here, but logging would be nice to add.
}


// All primitive (non-class types) methods are exactly the same as the ones used in a timed robot

void Arm::move(double setpoint){
    // Carbon copy of timed
    m_arm_pid.SetReference(setpoint,
                    rev::CANSparkMax::ControlType::kPosition);
    
    //OPTIONAL: add logging here
}

bool Arm::is_loaded(){
    // Carbon copy of timed

    frc::SmartDashboard::PutNumber("range", m_tof_sensor.GetRange());
    return (
        (units::millimeter_t {m_tof_sensor.GetRange()} < CONSTANTS::ROLLER::LOADED_RANGE)
    );
}

double Arm::get_position(){
    // Carbon copy of timed
    return m_arm_encoder.GetPosition();
}

void Arm::spin(double speed){
    // Carbon copy of timed
    m_left_roller_motor.Set(speed);
}


frc2::CommandPtr Arm::stow_command(){
    // This one is a bit weird, a cmdptr doesn't *do* anything, rather it gives the scheduler a thing to do.
    // This means that you can use .OnTrue(m_arm.stow_command()) and bind it to a button.

// A Run Command is a command that just keeps running untill it stops.
// Thankfuly, stowing is a task that just keeps running and doesn't need to end

return frc2::RunCommand(
    // frc2::RunCommand takes a lambda (std::function<void()>) which uses the below syntax:
    // [capture] -> return_type (paramaters) {body}
    // However, this won't be used in full, as that is needlessly verbose.

    // The square brackets are the capture, which deterimines what variables are in scope and can be used by the lamda
    // [this] is usualy a safe bet
    [this]

    {    
        // The curly brackets are the meat of the function, these do things.
        frc::SmartDashboard::PutString("state", "stow");        
        Arm::spin(CONSTANTS::FF_SPEED);
        Arm::move(CONSTANTS::ARM::STORE_POS);
    },
    //The final brackets act as a list of required subsystems, usualy all you need is {this}
    {this}
    ).ToPtr(); // .ToPtr() keeps the bindings just a little more consise

}

// Commands can accept parameters like any other function
frc2::CommandPtr Arm::extake_command(CONSTANTS::TARGET tgt)
{
    // A run command runs untill explicitly interputed.
    return
    frc2::RunCommand(
        // Any specific variables not captured by `this` must be added to the capture list
        // You could also do [=, this] or [=], but [=] is not recomended
        [this, tgt] {
        if (tgt == CONSTANTS::TARGET::LOW)
        {
            Arm::spin(CONSTANTS::ARM::EXTAKE_VEL_LOW);
        }
         else if (tgt == CONSTANTS::TARGET::MID)
         {
            Arm::spin(CONSTANTS::ARM::EXTAKE_VEL_MID);
         }

        },
        {this}
    ).ToPtr();
}

frc2::CommandPtr Arm::move_low_command()
{
    // A FunctionalCommand is a more verbose yet powerful way of doing commands. It takes four functions to make, but handles more on it's own.
        return frc2::FunctionalCommand(
        // The first function is the onInit function, which runs each time the command is scheduled
        [this] {},
        // The second function is the Execute function, which runs once per cycle when the command is running. This is where you do things
        [this] {
            Arm::move(CONSTANTS::ARM::SCORE_POS_LOW);
        },
        // This is the onEnd function, which runs once at 
        [this] (bool interupted) {},
        [this] { return (Arm::get_position() >= CONSTANTS::ARM::SCORE_POS_LOW*0.98) && (Arm::get_position() <= CONSTANTS::ARM::SCORE_POS_LOW*1.02);},
        {this}
    ).ToPtr();
}

frc2::CommandPtr Arm::move_mid_command()
{
        return frc2::FunctionalCommand(
        [this] {},
        [this] {
            Arm::move(CONSTANTS::ARM::SCORE_POS_LOW);
        },
        [this] (bool interupted) {},
        [this] { return (Arm::get_position() >= CONSTANTS::ARM::SCORE_POS_MID*0.98) && (Arm::get_position() <= CONSTANTS::ARM::SCORE_POS_MID*1.02);},
        {this}
    ).ToPtr();
}