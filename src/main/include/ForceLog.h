// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <string>
#include <iostream>
#include <frc/DataLogManager.h>
#include <frc/DriverStation.h>
#include <networktables/StringTopic.h>
#include <networktables/StringArrayTopic.h>

class ForceLog {
 public:
  ForceLog();

  /// @brief Info for after matches, only printed to NT and to file
  /// @param key The name of the data being logged
  /// @param value The data being logged
 void debug(std::string value);

  /// @brief Logs to a file and to STDOUT
  /// @param value 
  void warn(std::string value);

  /// @brief Logs everywhere, and in red
  /// @param key 
  /// @param value 
  void error(std::string value);

  private:
  std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("logs");
};
