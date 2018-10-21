package frc.team1318.robot.common.robotprovider;

import javafx.beans.property.DoubleProperty;
import javafx.beans.property.SimpleDoubleProperty;

public class FauxbotSolenoid extends FauxbotActuatorBase implements ISolenoid
{
    private DoubleProperty currentValueProperty;

    public FauxbotSolenoid(int port)
    {
        this(0, port);
    }

    public FauxbotSolenoid(int moduleNumber, int port)
    {
        FauxbotActuatorManager.set(port, this);

        this.currentValueProperty = new SimpleDoubleProperty();
        this.currentValueProperty.set(0.0);
    }

    public void set(boolean on)
    {
        if (on)
        {
            this.currentValueProperty.set(1.0);
        }
        else
        {
            this.currentValueProperty.set(0.0);
        }
    }

    public DoubleProperty getProperty()
    {
        return this.currentValueProperty;
    }
}