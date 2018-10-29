package frc.team1318.robot.common.robotprovider;

public abstract class FauxbotAdvancedMotorBase extends FauxbotMotorBase
{
    protected FauxbotAdvancedMotorBase(int deviceNumber)
    {
        FauxbotActuatorManager.set(new FauxbotActuatorConnection(FauxbotActuatorConnection.ActuatorConnector.CAN, deviceNumber), this);
    }
}
