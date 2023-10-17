// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "ForceLog.h"

ForceLog::ForceLog() 
{
    frc::DataLogManager::Start();
    frc::DriverStation::StartDataLog(frc::DataLogManager::GetLog());
}

void ForceLog::warn(std::string value)
{
    frc::DataLogManager::Log("\033[33m WARNING: "+value);
    std::vector<std::string> entry = table->GetStringArray("warnings", std::vector<std::string>(0));
    entry.push_back(value);
    table->PutStringArray("warnings", entry);
}

void ForceLog::error(std::string value)
{
    frc::DataLogManager::Log("\033[31m ERROR: "+value);
    std::vector<std::string> entry = table->GetStringArray("errors", std::vector<std::string>(0));
    entry.push_back(value);
    table->PutStringArray("errors", entry);
}

void ForceLog::debug(std::string value)
{
    std::vector<std::string> entry = table->GetStringArray("debug", std::vector<std::string>(0));
    entry.push_back(value);
    table->PutStringArray("debug", entry);

}


