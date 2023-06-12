//SwerveModule.h using example from wipilib
//intializes SwerveModule and their respective constants etc.

#pragma once

#include <numbers>

#include <frc/Encoder.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <units/angular_velocity.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>

class SwerveModule {
    public: 
        SwerveModule(
            int driveMotorChannel, int turningMotorChannel, int driveEncoderChannelA, int driveEncoderChannelB, int turningEncoderChannelA, int turningEncoderChannelB
        );
        frc::SwerveModuleState GetState() const;
        frc::SwerveModulePosition GetPosition() const;
        void SetDesiredState(const frc::SwerveModuleState& state);

    private:
        static constexpr double kWheelRadius = 0.5;
        static constexpr int kEncoderResolution = 1234;

        static constexpr auto kMaxAngularVelocity = std::numbers::pi * 1_rad_per_s;
        static constexpr auto kMaxAngularAcceleration = std::numbers::pi * 1_rad_per_s / 1_s; 

        frc::PWMSparkMax m_driveMotor;
        frc::PWMSparkMax m_turningMotor;

        frc::Encoder m_driveEncoder;
        frc::Encoder m_turningEncoder;

        frc2::PIDController m_drivePIDController{1.0, 0, 0};
        frc::ProfiledPIDController<units::radians> m_turningPIDController{ 1.0,0.0,0.0, {kMaxAngularVelocity, kMaxAngularAcceleration}};
        frc::SimpleMotorFeedforward<units::meters> m_driveFeedforward{1_V,3_V / 1_mps};
        frc::SimpleMotorFeedforward<units::radians> m_turnFeedforward{ 1_V, 0.5_V / 1_rad_per_s};
};
