//SwerveModule class used for individual swerve motors


#include <subsystems/SwerveModule.h>
#include <numbers>
#include <frc/geometry/Rotation2d.h>

//define SwerveModule

SwerveModule::SwerveModule (
  const int driveMotorChannel,
  const int turningMotorChannel,
  const int driveEncoderChannelA,
  const int driveEncoderChannelB,
  const int turningEncoderChannelA,
  const int turningEncoderChannelB
):  
  m_driveMotor(driveMotorChannel),
  m_turningMotor(turningMotorChannel),
  m_driveEncoder(driveEncoderChannelA, driveEncoderChannelB),
  m_turningEncoder(turningEncoderChannelA, turningEncoderChannelB)
{
  m_driveEncoder.SetDistancePerPulse(2 * std::numbers::pi * kWheelRadius / kEncoderResolution);

  m_turningEncoder.SetDistancePerPulse(2 * std::numbers::pi / kEncoderResolution);
}

frc::SwerveModuleState SwerveModule::GetState() const {
  return {
    units::meters_per_second_t{m_driveEncoder.GetRate()}, 
    units::radian_t{m_turningEncoder.GetDistance()}
  };
}

frc::SwerveModulePosition SwerveModule::GetPosition() const {
  return {
    units::meter_t{m_driveEncoder.GetDistance()}, 
    units::radian_t{m_turningEncoder.GetDistance()}
  };
}

void SwerveModule::SetDesiredState(const frc::SwerveModuleState& referenceState) {
  // Optimize the reference state to avoid spinning further than 90 degrees
    const auto state = frc::SwerveModuleState::Optimize
    (
    referenceState, units::radian_t{m_turningEncoder.GetDistance()}
    );

  // Calculate the drive output from the drive PID controller.
    const auto driveOutput = m_drivePIDController.Calculate
    (
    m_driveEncoder.GetRate(), state.speed.value()
    );

    const auto driveFeedforward = m_driveFeedforward.Calculate(state.speed);

  // Calculate the turning motor output from the turning PID controller.
    const auto turnOutput = m_turningPIDController.Calculate
    (
    units::radian_t{m_turningEncoder.GetDistance()}, state.angle.Radians()
    );

    const auto turnFeedforward = m_turnFeedforward.Calculate
    (
    m_turningPIDController.GetSetpoint().velocity
    );

  // Set the motor outputs.
    m_driveMotor.SetVoltage(units::volt_t{driveOutput} + driveFeedforward);
    m_turningMotor.SetVoltage(units::volt_t{turnOutput} + turnFeedforward);
}


