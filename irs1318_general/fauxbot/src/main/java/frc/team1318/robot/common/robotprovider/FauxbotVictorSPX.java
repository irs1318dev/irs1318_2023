package frc.team1318.robot.common.robotprovider;

import frc.team1318.robot.TuningConstants;

public class FauxbotVictorSPX extends FauxbotMotorBase implements IVictorSPX
{
    public FauxbotVictorSPX(int deviceNumber)
    {
        super(deviceNumber);
    }

    public void setControlMode(TalonSRXControlMode mode)
    {
    }

    public void setInvertOutput(boolean invert)
    {
    }

    public void setInvertSensor(boolean invert)
    {
    }
}
