package frc.team1318.robot.common.robotprovider;

import javafx.beans.property.DoubleProperty;
import javafx.beans.property.SimpleDoubleProperty;

public class FauxbotDoubleSolenoid extends FauxbotActuatorBase implements IDoubleSolenoid
{
    private DoubleProperty currentValueProperty;

    public FauxbotDoubleSolenoid(int forwardPort, int reversePort)
    {
        this(0, forwardPort, reversePort);
    }

    public FauxbotDoubleSolenoid(int moduleNumber, int forwardPort, int reversePort)
    {
        FauxbotActuatorManager.set(forwardPort, this);
        FauxbotActuatorManager.set(reversePort, null);

        this.currentValueProperty = new SimpleDoubleProperty();
        this.currentValueProperty.set(0.0);
    }

    public void set(DoubleSolenoidValue value)
    {
        if (value == DoubleSolenoidValue.kOff)
        {
            this.currentValueProperty.set(0.0);
        }
        else if (value == DoubleSolenoidValue.kForward)
        {
            this.currentValueProperty.set(1.0);
        }
        else if (value == DoubleSolenoidValue.kReverse)
        {
            this.currentValueProperty.set(-1.0);
        }
    }

    public DoubleSolenoidValue get()
    {
        if (this.currentValueProperty.get() > 0.0)
        {
            return DoubleSolenoidValue.kForward;
        }

        if (this.currentValueProperty.get() < 0.0)
        {
            return DoubleSolenoidValue.kReverse;
        }

        return DoubleSolenoidValue.kOff;
    }

    public DoubleProperty getProperty()
    {
        return this.currentValueProperty;
    }
}
