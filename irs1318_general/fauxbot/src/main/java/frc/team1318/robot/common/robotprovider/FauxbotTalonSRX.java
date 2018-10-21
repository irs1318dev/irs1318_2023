package frc.team1318.robot.common.robotprovider;

public class FauxbotTalonSRX extends FauxbotMotorBase implements ITalonSRX
{
    public FauxbotTalonSRX(int deviceNumber)
    {
        super(deviceNumber);
    }

    public void setControlMode(TalonSRXControlMode mode)
    {
    }

    public void setSensorType(TalonSRXFeedbackDevice feedbackDevice)
    {
    }

    public void setFeedbackFramePeriod(int periodMS)
    {
    }

    public void setPIDFFramePeriod(int periodMS)
    {
    }

    public void configureVelocityMeasurements()
    {
    }

    public void setSelectedSlot(int slotId)
    {
    }

    public void setPIDF(double p, double i, double d, double f, int slotId)
    {
    }

    public void setMotionMagicPIDF(double p, double i, double d, double f, int velocity, int acceleration, int slotId)
    {
    }

    public void setPIDF(double p, double i, double d, double f, int izone, double closeLoopRampRate, int slotId)
    {
    }

    public void setForwardLimitSwitch(boolean enabled, boolean normallyOpen)
    {
    }

    public void setReverseLimitSwitch(boolean enabled, boolean normallyOpen)
    {
    }

    public void setInvertOutput(boolean flip)
    {
    }

    public void setInvertSensor(boolean flip)
    {
    }

    public void setNeutralMode(TalonSRXNeutralMode neutralMode)
    {
    }

    public void setVoltageCompensation(boolean enabled, double maxVoltage)
    {
    }

    public void stop()
    {
    }

    public void setPosition(int position)
    {
    }

    public void reset()
    {
    }

    public int getPosition()
    {
        return 0;
    }

    public double getVelocity()
    {
        return 0.0;
    }

    public double getError()
    {
        return 0.0;
    }

    public TalonSRXLimitSwitchStatus getLimitSwitchStatus()
    {
        return null;
    }
}
