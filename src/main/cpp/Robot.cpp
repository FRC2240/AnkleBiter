// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include "Buttons.h"
#include <fmt/core.h>
#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit()
{
  m_chooser.AddOption(kScoreDoNothing, kScoreDoNothing);
  m_chooser.AddOption(kScoreGoBack, kScoreGoBack);
  m_chooser.AddOption(kScoreBalance, kScoreBalance);
  m_chooser.AddOption(kScoreCrossLineBalance, kScoreCrossLineBalance);
  m_chooser.AddOption(kScoreDock, kScoreDock);
  m_chooser.AddOption(kScoreCoral, kScoreCoral);
  //m_chooser.AddOption(kScoreIntake, kScoreIntake);
  // m_chooser.AddOption(kScoreMidDoNothing, kScoreMidDoNothing);
  // m_chooser.AddOption(kScoreMidCrossLine, kScoreMidCrossLine);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
#ifndef CFG_NO_DRIVEBASE
  m_drivetrain.init();
  m_odometry.putField2d();
#endif
}

/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW
 * Dashboard, remove all of the chooser code and uncomment the GetString line
 * to get the auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the
 * SendableChooser make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit()
{
  m_drivetrain.zero_yaw();
#ifndef CFG_NO_DRIVEBASE
  m_odometry.update();
  // m_drivetrain.flip();
#endif

  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  fmt::print("Auto selected: {}\n", m_autoSelected);
  m_score_timer.Start();
  if(m_autoSelected == "Score Go Back")
    {
      m_auto_sequence = &score_go_back;
      std::cout << "Score Go Back \n";
    }
  else if(m_autoSelected == "Score do nothing")
    {
      m_auto_sequence = &score_do_nothing;
      std::cout << "Score do nothing \n";
    }
  else if(m_autoSelected == "Score Balance")
    {
      m_auto_sequence = &score_balance;
      std::cout << "Score Balance \n";
    }
  else if(m_autoSelected == "Score cross line balance")
    {
      m_auto_sequence = &score_cross_line_bal;
      std::cout << "Score cross line balance \n";
    }
  else if(m_autoSelected == "Score and dock")
    {
      m_auto_sequence = &score_dock;
      std::cout << "Score mid cross line \n";
    }
    else if (m_autoSelected == "Coral Score")
    {
      m_auto_sequence = &score_coral;
      std::cout << "Coral Score \n";
    }
    /*else if (m_autoSelected == &)
    {

    }
*/
  // m_fallback_traj = m_trajectory.generate_live_traj(
  //   m_trajectory.fall_back(CONSTANTS::TRAJECTORY::fall_back_center));
  //   m_trajectory.init_live_traj(m_fallback_traj);
}

void Robot::AutonomousPeriodic()
{
  m_odometry.update();
  m_odometry.putField2d();
  frc::SmartDashboard::PutNumber("navx", m_drivetrain.getAngle().value());

  m_action = m_auto_sequence->front();
  switch(m_action)
    {
    case CONSTANTS::AUTO_ACTIONS::NOTHING:
       std::cout << "Do nothing! \n";
      break;

    case CONSTANTS::AUTO_ACTIONS::BALANCE:
      m_auto_balence.auto_balance_routine();
      break;

    case CONSTANTS::AUTO_ACTIONS::CENTER_CROSS_LINE:
      m_fallback_traj = m_trajectory.generate_live_traj(
          m_trajectory.fall_back(CONSTANTS::TRAJECTORY::fall_back_center));
      m_trajectory.init_live_traj(m_fallback_traj);
      m_action = CONSTANTS::AUTO_ACTIONS::CROSS_LINE_P;
      break;

    case CONSTANTS::AUTO_ACTIONS::CROSS_LINE:
      // std::cout << "Cross line! \n";
      m_fallback_traj = m_trajectory.generate_live_traj(
          m_trajectory.fall_back(CONSTANTS::TRAJECTORY::fall_back_dist));
      m_trajectory.init_live_traj(m_fallback_traj);
      m_auto_sequence->push_front(CONSTANTS::AUTO_ACTIONS::CROSS_LINE_P);
      break;

    case CONSTANTS::AUTO_ACTIONS::DOCK_CROSS_LINE:
      // std::cout << "DOCK! \n";
      m_fallback_traj = m_trajectory.generate_live_traj(
          m_trajectory.fall_back(CONSTANTS::TRAJECTORY::dock_dist));
      m_trajectory.init_live_traj(m_fallback_traj);
      m_auto_sequence->push_front(CONSTANTS::AUTO_ACTIONS::CROSS_LINE_P);
      break;

    case CONSTANTS::AUTO_ACTIONS::CROSS_LINE_P:
      // std::cout << "Cross line P! \n";
      if(m_trajectory.follow_live_traj(m_fallback_traj))
        {
          m_auto_sequence->pop_front();
          m_auto_sequence->push_front(CONSTANTS::AUTO_ACTIONS::NOTHING);
        }

      break;

    case CONSTANTS::AUTO_ACTIONS::SCORE:
      // std::cout << "Score! \n";
      if(m_score_timer.Get() <= 0.5_s)
        {
          m_roller.spin(-0.35);
        }
      else
        {
          // std::cout << "Time reached \n";
          m_auto_sequence->pop_front();
          m_action = m_auto_sequence->front();
          m_roller.spin(0);
          m_score_timer.Stop();
          m_score_timer.Reset();
        }
      break;

    case CONSTANTS::AUTO_ACTIONS::SCORE_MID:
      // std::cout << "Mid Score! \n";
      m_arm.move(CONSTANTS::ARM::SCORE_POS_HIGH);
      if(m_score_timer.Get() > 1.0_s)
        {
          m_roller.spin(-1);
        }
      if(m_score_timer.Get() >= 1.5_s)
        {
          // std::cout << "Time reached \n";
          m_auto_sequence->pop_front();
          m_action = m_auto_sequence->front();
          m_roller.spin(0);
          // m_score_timer.Stop();
          // m_score_timer.Reset();
        }
      break;
     case CONSTANTS::AUTO_ACTIONS::SCORE_INTAKE:
              m_drivetrain.drive(1_mps, 0_mps, units::radians_per_second_t{0}, false);

      m_arm.move(CONSTANTS::ARM::INTAKE_POS);
      m_roller.spin(CONSTANTS::ARM::INTAKE_VEL); // No idea if this is the
                                                 // right value/sign or not
      frc::SmartDashboard::PutBoolean("loaded", m_roller.is_loaded());
      if(m_roller.is_loaded())
        {
                        m_drivetrain.drive(0_mps, 0_mps, units::radians_per_second_t{0}, false);
          std::cout << "here\n";
          m_auto_sequence->pop_front();
          m_action = m_auto_sequence->front();
        }
      break;
      case CONSTANTS::AUTO_ACTIONS::STOWED:
        m_arm.move(CONSTANTS::ARM::STORE_POS);
        m_roller.spin(CONSTANTS::FF_SPEED);
        frc::SmartDashboard::PutString("state", "stowed");
        m_auto_sequence->pop_front();
        m_action = m_auto_sequence->front();
      break;

      case CONSTANTS::AUTO_ACTIONS::PATH_1:
      {    
        std::cout << "Path 1\n";    
        m_path_trajectory1 = m_trajectory.extract("3_cube_path_1", units::meters_per_second_t {1.77186}, units::meters_per_second_squared_t {3.54373});        
        m_trajectory.init_live_traj(m_path_trajectory1);        
        m_auto_sequence->pop_front();
        m_action = m_auto_sequence->front();
      break;
      }
    
    case CONSTANTS::AUTO_ACTIONS::PATH_2:
    {
      std::cout << "Path 2\n"; 
      m_path_trajectory2 = m_trajectory.extract("3_cube_path_2", units::meters_per_second_t {1.77186}, units::meters_per_second_squared_t {3.54373});
      m_trajectory.init_live_traj(m_path_trajectory2);
      m_auto_sequence->pop_front();
      m_action = m_auto_sequence->front();
      break;
    }

    case CONSTANTS::AUTO_ACTIONS::PATH_3:
    {
      std::cout << "Path 3\n"; 
      m_path_trajectory3 = m_trajectory.extract("3_cube_path_3", units::meters_per_second_t {1.77186}, units::meters_per_second_squared_t {3.54373});
      m_trajectory.init_live_traj(m_path_trajectory3);
      m_auto_sequence->pop_front();
      m_action = m_auto_sequence->front();
      break;
    }

    case CONSTANTS::AUTO_ACTIONS::PATH_4:
    {
      std::cout << "Path 4\n"; 
      m_path_trajectory4 = m_trajectory.extract("3_cube_path_4", units::meters_per_second_t {1.77186}, units::meters_per_second_squared_t {3.54373});
      m_trajectory.init_live_traj(m_path_trajectory4);
      m_auto_sequence->pop_front();
      m_action = m_auto_sequence->front();
      break;
    }

    case CONSTANTS::AUTO_ACTIONS::PATH_1_P:
    std::cout << "Path periodic 1\n"; 
     if (m_trajectory.follow_live_traj(m_path_trajectory1))
        {
          m_auto_sequence->pop_front();
          m_action = m_auto_sequence->front();
        }
      break;

    case CONSTANTS::AUTO_ACTIONS::PATH_2_P:
    std::cout << "Path periodic 2\n"; 
     if (m_trajectory.follow_live_traj(m_path_trajectory2))
        {
          m_auto_sequence->pop_front();
          m_action = m_auto_sequence->front();
        }
      break;

    case CONSTANTS::AUTO_ACTIONS::PATH_3_P:
    std::cout << "Path periodic 3\n"; 
     if (m_trajectory.follow_live_traj(m_path_trajectory3))
        {
          m_auto_sequence->pop_front();
          m_action = m_auto_sequence->front();
        }
      break;

    case CONSTANTS::AUTO_ACTIONS::PATH_4_P:
    std::cout << "Path periodic 4\n"; 
     if (m_trajectory.follow_live_traj(m_path_trajectory4))
        {
          m_auto_sequence->pop_front();
          m_action = m_auto_sequence->front();
        }
      break;

    case CONSTANTS::AUTO_ACTIONS::CORAL_SCORE:
      frc::SmartDashboard::PutNumber("navx", m_drivetrain.getAngle().value());
      is_driver_controled = !frc::SmartDashboard::GetBoolean("dbg/snap_zero", false);
      is_driver_controled = false;
      auto coral = m_vision.get_coral();
      if(coral) {
        frc::SmartDashboard::PutNumber("coral value", coral.value().value());
      frc::SmartDashboard::PutBoolean("seesgp", true);
        // Because Vision::get_coral() is of type
        // std::optional<units::degree_t>, running an
        // if/else checks if the value is valid
        
          // if (coral.value().value() < 1 && coral.value().value() > -1){
            std::cout << coral.value().value() << std::endl;
          m_drivetrain.face_direction(0_deg, coral.value().value());
          frc::SmartDashboard::PutNumber("angle of gp", coral.value().value());
          // }
          // A .value() on an std::option<units::degree_t> returns a
          // units::degree_t, not a double
            if (coral.value().value() < 2 && coral.value().value() > -2){
                        m_auto_sequence->pop_front();
          m_action = m_auto_sequence->front();
                       }
        }
        else
        {
          m_drivetrain.stop();
        }
        break;

      //default:
      
    }
}

void Robot::TeleopInit()
{
    frc::SmartDashboard::PutBoolean("dbg/bool_snap_coral", false);

  frc::SmartDashboard::PutBoolean("dbg/snap_zero", false);
  frc::SmartDashboard::PutBoolean("dbg/bool_snap_angle", false);
  frc::SmartDashboard::PutNumber("dbg/val_snap_angle", 0.0);
#ifndef CFG_NO_DRIVEBASE
  m_odometry.update();
#endif
is_driver_controled = true;
}

void Robot::swerveDrive(bool const &field_relative)
{
#ifndef CFG_NO_DRIVEBASE
  frc::SmartDashboard::PutNumber("Pitch", m_drivetrain.get_pitch());
  frc::SmartDashboard::PutNumber("X", m_drivetrain.acc.GetX());
  frc::SmartDashboard::PutNumber("Y", m_drivetrain.acc.GetY());
  frc::SmartDashboard::PutNumber("Z", m_drivetrain.acc.GetZ());
  frc::SmartDashboard::PutNumber("gryo", m_drivetrain.getAngle().value());

  if(BUTTON::stick.GetStartButtonReleased())
    {
      m_drivetrain.zero_yaw();
    }
  const units::meters_per_second_t left_right{
    -(frc::ApplyDeadband(BUTTON::DRIVETRAIN::LX(), CONSTANTS::DEADBAND))
    * CONSTANTS::DRIVE::TELEOP_MAX_SPEED
  };

  const units::meters_per_second_t front_back{
    -(frc::ApplyDeadband(BUTTON::DRIVETRAIN::LY(), CONSTANTS::DEADBAND))
    * CONSTANTS::DRIVE::TELEOP_MAX_SPEED
  };
  auto const rot
      = frc::ApplyDeadband(BUTTON::DRIVETRAIN::RX(), CONSTANTS::DEADBAND)
        * (CONSTANTS::DRIVE::TELEOP_MAX_ANGULAR_SPEED);

  m_drivetrain.drive(front_back, -left_right, rot, field_relative);

  // frc::SmartDashboard::PutNumber("Gyro: ", m_drivetrain.getAngle().value());
  // frc::SmartDashboard::PutNumber("front/back: ", front_back.value());
  // frc::SmartDashboard::PutNumber("left/right: ", left_right.value());
#endif
}

void Robot::TeleopPeriodic()
{
  /*frc::SmartDashboard::PutNumber("navx", m_drivetrain.getAngle().value());
is_driver_controled = !frc::SmartDashboard::GetBoolean("dbg/snap_zero", false);

  if(frc::SmartDashboard::GetBoolean("dbg/snap_zero", false) == true)
    {
      is_driver_controled = false;
      if (m_drivetrain.face_direction(0_deg))
      {
        is_driver_controled = true;
      }
    }

  if(frc::SmartDashboard::GetBoolean("dbg/bool_snap_angle", false) == true)
    {
      units::degree_t angle{ frc::SmartDashboard::GetNumber(
          "dbg/val_snap_angle", 0.0) };
      m_drivetrain.faceDirection(0_mps, 0_mps, angle, false, 0.0);
    }

  else if(frc::SmartDashboard::GetBoolean("dbg/bool_snap_coral", false) == true)
    { 
      is_driver_controled = false;
      auto coral = m_vision.get_coral();
 
      if(coral) {
        frc::SmartDashboard::PutNumber("coral value", coral.value().value());
      frc::SmartDashboard::PutBoolean("seesgp", true);
        // Because Vision::get_coral() is of type
        // std::optional<units::degree_t>, running an
        // if/else checks if the value is valid
        
          // if (coral.value().value() < 1 && coral.value().value() > -1){
            std::cout << coral.value().value() << std::endl;
          m_drivetrain.face_direction(0_deg, coral.value().value());
          frc::SmartDashboard::PutNumber("angle of gp", coral.value().value());
          // }
          // A .value() on an std::option<units::degree_t> returns a
          // units::degree_t, not a double
        }
        else
        {
          m_drivetrain.stop();
        }
    //}*/
if (is_driver_controled)
{
  Robot::swerveDrive(true);
}
  if(BUTTON::STOWED())
    {
      m_stowed_toggle = !m_stowed_toggle;\
      m_intake_toggle = 0;
      m_intake_overide_toggle = 0;
      m_extake_low_toggle = 0;
      m_extake_mid_toggle = 0;
      m_extake_high_toggle = 0;
    }
  if(BUTTON::INTAKE())
    {
      m_stowed_toggle = 0;
      m_intake_toggle = !m_intake_toggle;
      m_intake_overide_toggle = 0;
      m_extake_low_toggle = 0;
      m_extake_mid_toggle = 0;
      m_extake_high_toggle = 0;
    }
  if(BUTTON::EXTAKE_LOW() && m_state != CONSTANTS::STATE::EXTAKE_HIGH)
    {
      m_stowed_toggle = 0;
      m_intake_toggle = 0;
      m_intake_overide_toggle = 0;
      m_extake_low_toggle = !m_extake_low_toggle;
      m_extake_mid_toggle = 0;
      m_extake_high_toggle = 0;
      m_score_timer.Reset();
    }
  if(BUTTON::EXTAKE_MID())
    {
      m_stowed_toggle = 0;
      m_intake_toggle = 0;
      m_intake_overide_toggle = 0;
      m_extake_low_toggle = 0;
      m_extake_mid_toggle = !m_extake_mid_toggle;
      m_extake_high_toggle = 0;
      m_score_timer.Reset();
    }
  if(BUTTON::EXTAKE_HIGH())
    {
      m_stowed_toggle = 0;
      m_intake_toggle = 0;
      m_intake_overide_toggle = 0;
      m_extake_high_toggle = !m_extake_high_toggle;
      m_extake_low_toggle = 0;
      m_extake_mid_toggle = 0;
      m_score_timer.Reset();
    }
  if(BUTTON::INTAKE_OVERIDE())
    {
      m_stowed_toggle = 0;
      m_intake_toggle = 0;
      m_intake_overide_toggle = !m_intake_overide_toggle;
      m_extake_low_toggle = 0;
      m_extake_mid_toggle = 0;
      m_extake_high_toggle = 0;
    }

  if(m_extake_low_toggle)
    {
      m_state = CONSTANTS::STATE::EXTAKE_LOW;
    }
  if(m_intake_toggle)
    {
      m_state = CONSTANTS::STATE::INTAKE;
    }
  if(m_stowed_toggle)
    {
      m_state = CONSTANTS::STATE::STOWED;
    }
  if(m_extake_mid_toggle)
    {
      m_state = CONSTANTS::STATE::EXTAKE_MID;
    }
  if(m_extake_high_toggle)
    {
      m_state = CONSTANTS::STATE::EXTAKE_HIGH;
    }
  if(m_intake_overide_toggle)
    {
      m_state = CONSTANTS::STATE::INTAKE_OVERIDE;
    }
  switch(m_state)
    {
    case CONSTANTS::STATE::STOWED:
      m_arm.move(CONSTANTS::ARM::STORE_POS);
      m_roller.spin(CONSTANTS::FF_SPEED);
      frc::SmartDashboard::PutString("state", "stowed");

      break;

    case CONSTANTS::STATE::INTAKE:
      frc::SmartDashboard::PutString("state", "intake");
      m_arm.move(CONSTANTS::ARM::INTAKE_POS);
      m_roller.spin(CONSTANTS::ARM::INTAKE_VEL); // No idea if this is the
                                                 // right value/sign or not
      frc::SmartDashboard::PutBoolean("loaded", m_roller.is_loaded());
      if(m_roller.is_loaded())
        {
          m_intake_toggle = 0;
          m_stowed_toggle = 1;
          m_state = CONSTANTS::STATE::STOWED;
        }
      break;

    case CONSTANTS::STATE::INTAKE_OVERIDE:
      frc::SmartDashboard::PutString("state", "intake");
      m_arm.move(CONSTANTS::ARM::INTAKE_POS);
      m_roller.spin(CONSTANTS::ARM::INTAKE_VEL);
      break;

    case CONSTANTS::STATE::EXTAKE_LOW:
      m_arm.move(CONSTANTS::ARM::SCORE_POS_LOW);
      m_roller.spin(-0.35);
      m_score_timer.Start();

      if(m_score_timer.Get() > units::time::second_t(0.5))
        {
          m_score_timer.Stop();
          m_score_timer.Reset();
          m_state = CONSTANTS::STATE::STOWED;
          m_extake_low_toggle = !m_extake_low_toggle;
        }
      break;

    case CONSTANTS::STATE::EXTAKE_MID:
      m_arm.move(CONSTANTS::ARM::SCORE_POS_MID);
      m_score_timer.Start();

      if(m_score_timer.Get() > units::time::second_t(1.0))
        {
          m_roller.spin(-1);
        }
      if(m_score_timer.Get() > units::time::second_t(1.5))
        {
          m_score_timer.Stop();
          m_score_timer.Reset();
          m_state = CONSTANTS::STATE::STOWED;
          m_extake_mid_toggle = !m_extake_mid_toggle;
        }
      break;

    case CONSTANTS::STATE::EXTAKE_HIGH:
      m_arm.move(CONSTANTS::ARM::SCORE_POS_HIGH);

      if(BUTTON::EXECUTE_EXTAKE())
        {
          m_score_timer.Start();
          m_roller.spin(-1);
        }
      if(m_score_timer.Get() > units::time::second_t(0.5))
        {
          m_score_timer.Stop();
          m_score_timer.Reset();
          m_state = CONSTANTS::STATE::STOWED;
          m_extake_high_toggle = !m_extake_high_toggle;
        }
      break;
    }
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() 
{
  frc::SmartDashboard::PutNumber("navx", m_drivetrain.getAngle().value());

  if(frc::SmartDashboard::GetBoolean("dbg/snap_zero", false) == true)
    {
      is_driver_controled = false;
      if (m_drivetrain.face_direction(0_deg))
      {
        is_driver_controled = true;
      }
    }

  if(frc::SmartDashboard::GetBoolean("dbg/bool_snap_angle", false) == true)
    {
      units::degree_t angle{ frc::SmartDashboard::GetNumber(
          "dbg/val_snap_angle", 0.0) };
      m_drivetrain.faceDirection(0_mps, 0_mps, angle, false, 0.0);
    }

  if(frc::SmartDashboard::GetBoolean("dbg/bool_snap_doral", false) == true)
    {
      if(m_vision.get_coral())
        // Because Vision::get_coral() is of type
        // std::optional<units::degree_t>, running an
        // if/else checks if the value is valid
        {
          m_drivetrain.faceDirection(0_mps, 0_mps,
                                     m_vision.get_coral().value(), false, 0.0);
          // A .value() on an std::option<units::degree_t> returns a
          // units::degree_t, not a double
        }
    }

}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
