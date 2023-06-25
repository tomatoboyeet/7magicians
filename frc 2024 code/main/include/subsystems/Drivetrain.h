#pragma once

#include <numbers>

#include <frc/AnalogGyro.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>

#include "SwerveModule.h"
#include <Constants.h>

using namespace OperatorConstants;

/**
 * Represents a swerve drive style drivetrain.
 */
class Drivetrain {
public:
    Drivetrain() {
        m_gyro.Reset();
    }

    void Drive(
        units::meters_per_second_t xSpeed,
        units::meters_per_second_t ySpeed,
        units::radians_per_second_t rot,
        bool fieldRelative
    );

    void UpdateOdometry();

private:
    //use kHalfChassisSize if the Chassis is symmetrical
    frc::Translation2d m_frontLeftLocation {+kHalfChassisSize, +kHalfChassisSize};
    frc::Translation2d m_frontRightLocation {+kHalfChassisSize, -kHalfChassisSize};
    frc::Translation2d m_backLeftLocation {-kHalfChassisSize, +kHalfChassisSize};
    frc::Translation2d m_backRightLocation {-kHalfChassisSize, -kHalfChassisSize};
    
    //channels (driveM, turnM, driveEncA, driveEncB, turnEncA, turnEncB)
    SwerveModule m_frontLeft {kFLdriveM, kFLturnM, kFLdriveEA, kFLdriveEB, kFLturnEA, kFLturnEB};
    SwerveModule m_frontRight {kFRdriveM, kFRturnM, kFRdriveEA, kFRdriveEB, kFRturnEA, kFRturnEB};
    SwerveModule m_backLeft {kBLdriveM, kBLturnM, kBLdriveEA, kBLdriveEB, kBLturnEA, kBLturnEB};
    SwerveModule m_backRight {kBRdriveM, kBRturnM, kBRdriveEA, kBRdriveEB, kBRturnEA, kBRturnEB};

    frc::AnalogGyro m_gyro{0};

    frc::SwerveDriveKinematics<4> m_kinematics{
        m_frontLeftLocation, 
        m_frontRightLocation, 
        m_backLeftLocation,
        m_backRightLocation
    };

    frc::SwerveDriveOdometry<4> m_odometry{
        m_kinematics,
        m_gyro.GetRotation2d(),
        {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
        m_backLeft.GetPosition(), m_backRight.GetPosition()}
    };
};