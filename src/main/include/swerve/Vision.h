#pragma once

#include "Constants.h"
#include <cmath>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/smartdashboard/Field2d.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <units/angle.h>
#include <units/length.h>
#include <vector>
// #include "frc/smartdashboard/Smartdashboard.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"
// #include "wpi/span.h"

#include "swerve/Drivetrain.h"
#include "swerve/Odometry.h"
#include <chrono>
#include <iostream>
#include <numeric>
#include <thread>

#ifndef CFG_NO_DRIVEBASE
class Vision
{
public:
  Vision(Drivetrain *drivetrain, Odometry *odometry);
  ~Vision();

  int test = 10;

  /*
   * @brief A struct representing a single frame taken from a camera,
   * representing position.
   */
  struct Data
  {
    double trans_x;
    double trans_y;
    double rot_x;
    bool is_good; // Determine if data was written

    Data(){};

    Data(std::vector<double> in_vec)
    {
      if(in_vec.size() >= 6)
        {
          trans_x = in_vec[0];
          trans_y = in_vec[1];
          rot_x = in_vec[5];
        }
    };
  };

  std::optional<units::degree_t> get_coral();
  void get_raw_data(int i);

    /*
   * @brief The entrypoint into the vision system.
   * Should be called once every cycle to handle the entirety of the vision
   * system.
   *
   * @return If vision updated recently.
   * If true, then odometry was updated last cycle, if false it is still
   * updating.
   */
  bool pose_loop();

  double standard_dev(std::vector<double> v);

private:
  Drivetrain *m_drivetrain;
  Odometry *m_odometry;

  /// @brief Tables for the left camera's data to be stored in.
  std::vector<Vision::Data> m_left_buffer{ CONSTANTS::VISION::BUFFER_SIZE };
  /// @brief Tables for the right camera's data to be stored in.
  std::vector<Vision::Data> m_right_buffer{ CONSTANTS::VISION::BUFFER_SIZE };

  /// @brief Data that is assured to cause the standard deviation check to
  /// fail.
  Data nonsense;

  /// @brief Where in the circular buffer the program is operating on
  int m_index_pt = 0;
  std::vector<double> m_zero_vector = { 42.0, 42.0, 42.0, 92, 10, 22 };

  std::shared_ptr<nt::NetworkTable> m_limelight
      = nt::NetworkTableInstance::GetDefault().GetTable("limelight-dev");

  /*  std::shared_ptr<nt::NetworkTable> m_left_table
      = nt::NetworkTableInstance::GetDefault().GetTable("limelight-left");

std::shared_ptr<nt::NetworkTable> m_right_table
= nt::NetworkTableInstance::GetDefault().GetTable("limelight-right");
*/
  /**
   * @brief Converts from a table of data to a vector
   * @param f the table value to be collected (such as trans_x)
   * @param v the table to be operated on
   *
   * @return a vector of only the values defined in f.
   * Will filter out bad data.
   */
  std::vector<double> collect(double Data::*f, std::vector<Data> const &v);

  /**
   * @brief updates the odometry for the robot
   * @param bot_pose the values to update to
   */
  void update_pose(Data bot_pose);

  /**
   * @brief checks to see if the standard deviation is within a known threshold
   * @param buffer the data to be operated on
   * @return True if within threshold. False if not.
   */
  bool check_std_dev(std::vector<Data> buffer);

  /**
   * @brief A function to determine how many elements in the vector were taken
   * from the apriltags as opposed to filler data or nulls.
   *
   * @param data the dataset to check
   * @return the number of elements that were taken from apriltags.
   */
  int find_good_frames(std::vector<Data> data);

  /**
   * @brief finds the average of two vectors.
   * @param a first vector to average
   * @param b second vector to average
   * @return the average vector
   */
  double average(std::vector<double> buffer_a, std::vector<double> buffer_b);

  /**
   * @brief takes the average of one vector. Exactly what is says on the tin.
   */
  double average(std::vector<double> v);
};
#endif
