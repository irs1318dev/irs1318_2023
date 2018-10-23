package frc.team1318.robot.common.robotprovider;

public abstract class FauxbotSimpleMotorBase extends FauxbotMotorBase
{
    protected FauxbotSimpleMotorBase(int port)
    {
        FauxbotActuatorManager.set(new FauxbotActuatorConnection(FauxbotActuatorConnection.ActuatorConnector.PWM, port), this);
    }
}
