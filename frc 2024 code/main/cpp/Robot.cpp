// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/MathUtil.h>
#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <frc/filter/SlewRateLimiter.h>

#include <subsystems/Drivetrain.h>

class Robot : public frc::TimedRobot {
  public:
  void AutonomousPeriodic() override {
    DriveWithJoystick(false);
    m_swerve.UpdateOdometry();
  }

  void TeleopPeriodic() override { 
    DriveWithJoystick(true); 
  }

  void DriveWithJoystick(bool fieldRelative)
  {
    const auto xSpeed = -m_xspeedLimiter.Calculate(
      frc::ApplyDeadband(m_controller.GetLeftY(), 0.02)) * kMaxSpeed;

    const auto ySpeed = -m_yspeedLimiter.Calculate(
      frc::ApplyDeadband(m_controller.GetLeftX(), 0.02)) * kMaxSpeed;

    const auto rot = -m_rotLimiter.Calculate(
      frc::ApplyDeadband(m_controller.GetRightX(), 0.02)) * kMaxAngularSpeed;

    m_swerve.Drive(xSpeed, ySpeed, rot, fieldRelative);
  }

  private:
  frc::XboxController m_controller{0};
  Drivetrain m_swerve;

  frc::SlewRateLimiter<units::scalar> m_xspeedLimiter{3 / 1_s};
  frc::SlewRateLimiter<units::scalar> m_yspeedLimiter{3 / 1_s};
  frc::SlewRateLimiter<units::scalar> m_rotLimiter{3 / 1_s};
};

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
