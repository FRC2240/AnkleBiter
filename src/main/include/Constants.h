#pragma once

#include <units/length.h>
#include <units/angle.h>
#include <frc/DriverStation.h>
#include <iostream>
#include <vector>
#include <units/time.h>
#include <units/velocity.h>
#include <units/acceleration.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <ctre/Phoenix.h>
#include <numbers>

namespace CONSTANTS 
{
  constexpr int XBOX_PORT = 0;
  constexpr int AUX_XBOX_PORT = 1;
  constexpr double NON_TURBO = 1;
  constexpr double DEADBAND = 0.15;
  constexpr bool DEBUGGING = true; //DO NOT USE IN COMP.
  //SLOWS DOWN EVERYTHING & MAY CAUSE WATCHDOG EXEPTIONS.

  struct PidCoeff
  {
    const double 
    p = .0, /// Proportional. Based off distance from setpoint, most important but high values cause oscilation.
    i = .0, /// Integral. Based off the error over time, try to avoid if possible.
    d = .0, /// Derivative. Based off velocity from setpoint, can be used to smooth out oscilation caused by high P values.
    ff = .0, /// Feed Forward. Provides a constant boost to the output. Used to fight gravity or similar things.
    iz = .0, /// I Zone. A deadband (distance from zero) for when I takes effect. Try to avoid if possible.
    min = .0, /// Minimum output for control loop.
    max = .0;/// Maximum output for control loop.
  };

namespace ARM
{
  /// How large the increace should be for manual control.
  constexpr double STEP = 10;
  constexpr int LEFT_MOTOR_ID = 1;
  constexpr int RIGHT_MOTOR_ID = 2;
  constexpr double STORE_POS = 0.0; //CHANGEME
  constexpr double INTAKE_POS = 0.0; //CHANGEME
  constexpr double SCORE_POS = 0.0; //CHANGEME
/// The needed current for the motor to indicate a cube being stored
  constexpr double LOADED_CURRENT = 10;
}

  namespace ROLLER
  {
    constexpr int LEFT_MOTOR_ID = 3;
    constexpr int RIGHT_MOTOR_ID = 4;
    constexpr double VELOCITY = 1.0;
  }

  namespace DRIVE
  {
    constexpr units::meters_per_second_t ROBOT_MAX_SPEED =  14.533_fps;
    constexpr units::radians_per_second_t ROBOT_MAX_ANGULAR_SPEED{std::numbers::pi*1.25};
    constexpr units::meters_per_second_t TELEOP_MAX_SPEED = ROBOT_MAX_SPEED;
    constexpr units::radians_per_second_t TELEOP_MAX_ANGULAR_SPEED{std::numbers::pi*1.25};
    constexpr units::meters_per_second_t TRAJ_MAX_SPEED = ROBOT_MAX_SPEED;
    constexpr units::acceleration::meters_per_second_squared_t TRAJ_MAX_ACCELERATION = TRAJ_MAX_SPEED / 0.5_s;
    constexpr units::radians_per_second_t TRAJ_MAX_ANGULAR_SPEED = CONSTANTS::DRIVE::ROBOT_MAX_ANGULAR_SPEED;
    constexpr units::radians_per_second_squared_t TRAJ_MAX_ANGULAR_ACCELERATION{std::numbers::pi};
  }

  namespace TRAJECTORY
  {
    constexpr auto HP_VEL = 0.4_mps;
    constexpr units::meter_t SIMPLE_FORWARDS = -22_in; //CHANGEME
    const std::vector<units::meter_t> Y_POS =
      {
        /*
         * A list of all Y positions to score at.
         * Ordered from the pipe farthest on the robot's left side of the
         * red alliance grid to the rightmost.
         *
         * Since the field isn't mirrored, the lists are the same for the
         * red alliance and the blue alliance.
         *
         *  Copyright Westly Miller, 2023.
         */
        38.386_in,
        //16.386_in, //cube
        -5.614_in,
        -27.614_in,
        //-49.614_in, //cube
        -71.614_in,
        -93.614_in,
        //-115.614_in, //cube
        -137.614_in
      };
    namespace R
    {
      //Red Team and blue team will use seperate data.

      constexpr units::meter_t HIGH_X = 6.50_m;
      constexpr units::meter_t MID_X = 6.12_m;
      constexpr units::meter_t LOW_X = 6.09_m;
      constexpr units::meter_t HP = -6.6_m;
    }

    namespace B
    {
      constexpr units::meter_t HIGH_X = -6.50_m;
      constexpr units::meter_t MID_X = -6.12_m;
      constexpr units::meter_t LOW_X = -6.06_m;
      constexpr units::meter_t HP = 6.6_m;
    }
  }

  namespace VISION {
    //Remove above warning when values found
    constexpr int APRILTAG_PIPE = 1; 
    //Remove above warning when values found
    constexpr int BUFFER_SIZE = 14;
    constexpr int MIN_GOOD_FRAMES = 9;
    constexpr double MAX_STD_DEV = 0.04;
    constexpr double MIN_STD_DEV = 1.0e-16;
    constexpr double MAX_STD_DEV_ROT = 10; //CHANGEME
    constexpr double MIN_STD_DEV_ROT = 1.0e-10; //CHANGEME

  }
}
