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

class Arm : public frc2::SubsystemBase {
  public:
  Arm();

    void move(double setpoint);
 
 rev::SparkMaxPIDController m_arm_pid = m_left_arm_motor.GetPIDController();

  rev::SparkMaxAbsoluteEncoder m_arm_encoder = m_left_arm_motor.GetAbsoluteEncoder(rev::SparkMaxAbsoluteEncoder::Type::kDutyCycle);

    /**
   * @return True if there is a cube in the intake, as measured by a time of flight sensor
   */
    bool is_loaded();

/**
   * Will be called periodically whenever the CommandScheduler runs.
   */

    /**
   * Will be called periodically whenever the CommandScheduler runs during
   * simulation.
   */
  void SimulationPeriodic() override;

    double get_position();

  private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  // Note: The left motor will be the leader and the right will be the follower
  rev::CANSparkMax m_left_arm_motor {CONSTANTS::ARM::LEFT_MOTOR_ID, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
  rev::CANSparkMax m_right_arm_motor {CONSTANTS::ARM::RIGHT_MOTOR_ID, rev::CANSparkMaxLowLevel::MotorType::kBrushless};


  


  CONSTANTS::PidCoeff m_arm_coeff = {2, 0, 0, 0, 0, -1.0, 1.0};
};
