// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include "Constants.h"
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <rev/AbsoluteEncoder.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include "Constants.h"
#include <TimeOfFlight.h>
#include <units/length.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/FunctionalCommand.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/RepeatCommand.h>


class Arm : public frc2::SubsystemBase {
 public:
  Arm();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void move(double setpoint);

  bool is_loaded();

  double get_position();

  void spin(double speed);

  frc2::CommandPtr move_low_command();

  frc2::CommandPtr stow_command();

  frc2::CommandPtr extake_command(CONSTANTS::TARGET tgt);

  frc2::CommandPtr move_mid_command();


 private:
   rev::CANSparkMax m_left_arm_motor {CONSTANTS::ARM::LEFT_MOTOR_ID, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
  rev::CANSparkMax m_right_arm_motor {CONSTANTS::ARM::RIGHT_MOTOR_ID, rev::CANSparkMaxLowLevel::MotorType::kBrushless};


  rev::SparkMaxPIDController m_arm_pid = m_left_arm_motor.GetPIDController();

  rev::SparkMaxAbsoluteEncoder m_arm_encoder = m_left_arm_motor.GetAbsoluteEncoder(rev::SparkMaxAbsoluteEncoder::Type::kDutyCycle);


  CONSTANTS::PidCoeff m_arm_coeff = {2, 0, 0, 0, 0, -1.0, 1.0};


    frc::TimeOfFlight m_tof_sensor {CONSTANTS::ROLLER::TOF_CAN};

    rev::CANSparkMax m_left_roller_motor {CONSTANTS::ROLLER::LEFT_MOTOR_ID, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
    rev::CANSparkMax m_right_roller_motor {CONSTANTS::ROLLER::RIGHT_MOTOR_ID, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
    
    rev::SparkMaxPIDController m_roller_pid = m_left_roller_motor.GetPIDController();
    
    rev::SparkMaxRelativeEncoder m_roller_encoder = m_left_roller_motor.GetEncoder();

};
