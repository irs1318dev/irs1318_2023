package frc.robot.common.robotprovider;

public class FauxbotTalonFX extends FauxbotTalonXBase implements ITalonFX
{
    public FauxbotTalonFX(int deviceNumber)
    {
        super(deviceNumber);
    }

    @Override
    public void setSupplyCurrentLimit(boolean enabled, double currentLimit, double triggerThresholdCurrent, double triggerThresholdTime)
    {
    }
}
