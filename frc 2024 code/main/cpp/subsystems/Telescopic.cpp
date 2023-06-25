//Telescopic Part of the ArmModule

#include <Constants.h>
#include <subsystems/Telescopic.h>
#include <numbers>

using namespace OperatorConstants;

Telescopic::Telescopic(
    const int TelescopicMotorChannel,
    const int TelescopicEncoderChannelA,
    const int TelescopicEncoderChannelB
):
m_telescopicMotor(TelescopicMotorChannel),
m_telescopicEncoder(TelescopicEncoderChannelA, TelescopicEncoderChannelB)
{
    m_telescopicEncoder.SetDistancePerPulse(kThreadedGantry / kEncoderResolution);
}

units::meters_per_second_t Telescopic::GetState(){
    return units::meters_per_second_t{m_telescopicEncoder.GetRate()};
};

units::meter_t Telescopic::GetPosition(){
    return units::meter_t{m_telescopicEncoder.GetDistance()};
};

void Telescopic::SetDesiredState(double setPoint){
    //voltage for motor
    const auto driveOutput = m_telescopicPIDController.Calculate
    (
    m_telescopicEncoder.GetRate(), setPoint
    );

    const auto m_telescopicforward = m_telescopicFeedforward.Calculate(units::meters_per_second_t{m_telescopicEncoder.GetRate()});

    m_telescopicMotor.SetVoltage(units::volt_t{driveOutput} + m_telescopicforward);
};
