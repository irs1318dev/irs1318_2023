package frc.robot.common;

public class ComplementaryFilter
{
    private final double kO;
    private final double kN;

    private double currentValue;

    public ComplementaryFilter(double kO, double kN)
    {
        this(kO, kN, 0.0);
    }

    public ComplementaryFilter(double kO, double kN, double startingValue)
    {
        this.kO = kO;
        this.kN = kN;

        this.currentValue = startingValue;
    }

    public double getValue()
    {
        return this.currentValue;
    }

    public void update(double value)
    {
        if (this.currentValue != Double.NaN)
        {
            this.currentValue = this.currentValue * this.kO + value * this.kN;
        }
        else
        {
            this.currentValue = value * this.kN;
        }
    }

    public void reset()
    {
        this.currentValue = 0.0;
    }
}
