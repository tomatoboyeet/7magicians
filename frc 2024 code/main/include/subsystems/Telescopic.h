#pragma once


#include <frc/Encoder.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <Constants.h>

class Telescopic {
    public:
    Telescopic(
        int TelescopicMotorChannel, int TelescopicEncoderChannelA, int TelescopicEncoderChannelB
    );
    units::meters_per_second_t Telescopic::GetState();
    units::meter_t Telescopic::GetPosition();
    void Telescopic::SetDesiredState(double setPoint);

    private:
    frc::PWMSparkMax m_telescopicMotor;
    frc::Encoder m_telescopicEncoder;

    frc2::PIDController m_telescopicPIDController{1.0, 0, 0};
    frc::SimpleMotorFeedforward<units::meters> m_telescopicFeedforward{1_V, 3_V / 1_mps};
};