// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <numbers>
#include <units/angular_velocity.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

namespace OperatorConstants {

constexpr int kDriverControllerPort = 0;

static constexpr double kWheelRadius = 1;
static constexpr auto kMaxAngularVelocity = std::numbers::pi * 1_rad_per_s;
static constexpr auto kMaxAngularAcceleration = std::numbers::pi * 1_rad_per_s / 1_s;
static constexpr int kEncoderResolution = 1234;

//overall stats for robot.cpp
static constexpr units::meters_per_second_t kMaxSpeed = 3.0_mps; //driving 3 mps
static constexpr units::radians_per_second_t kMaxAngularSpeed{ std::numbers::pi }; //turning 1/2 rotations per second
static constexpr units::length::meter_t kHalfChassisSize = 0.381_m; //use if chassis is a square and swerve modules are symmetrical placed

//channels
//front left swerve module
static constexpr int kFLdriveM  = 1;  
static constexpr int kFLturnM   = 2;
static constexpr int kFLdriveEA = 0;
static constexpr int kFLdriveEB = 1;
static constexpr int kFLturnEA  = 2;
static constexpr int kFLturnEB  = 3;

//front right swerve module
static constexpr int kFRdriveM  = 1;  
static constexpr int kFRturnM   = 2;
static constexpr int kFRdriveEA = 0;
static constexpr int kFRdriveEB = 1;
static constexpr int kFRturnEA  = 2;
static constexpr int kFRturnEB  = 3;

//back left swerve module
static constexpr int kBLdriveM  = 1; 
static constexpr int kBLturnM   = 2;
static constexpr int kBLdriveEA = 0;
static constexpr int kBLdriveEB = 1;
static constexpr int kBLturnEA  = 2;
static constexpr int kBLturnEB  = 3;

//back right swerve module
static constexpr int kBRdriveM  = 1;  
static constexpr int kBRturnM   = 2;
static constexpr int kBRdriveEA = 0;
static constexpr int kBRdriveEB = 1;
static constexpr int kBRturnEA  = 2;
static constexpr int kBRturnEB  = 3;

}  // namespace OperatorConstants
