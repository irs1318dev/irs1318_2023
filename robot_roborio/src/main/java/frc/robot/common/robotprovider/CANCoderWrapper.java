package frc.robot.common.robotprovider;

import com.ctre.phoenix.sensors.*;

public class CANCoderWrapper implements ICANCoder
{
    private final CANCoder wrappedObject;

    public CANCoderWrapper(int deviceNumber)
    {
        this.wrappedObject = new CANCoder(deviceNumber);
    }

    public double getPosition()
    {
        return this.wrappedObject.getPosition();
    }

    public double getVelocity()
    {
        return this.wrappedObject.getVelocity();
    }

    public double getAbsolutePosition()
    {
        return this.wrappedObject.getAbsolutePosition();
    }

    public void setPosition(double newPosition)
    {
        this.wrappedObject.setPosition(newPosition);
    }

    public void configSensorDirection(boolean clockwisePositive)
    {
        this.wrappedObject.configSensorDirection(clockwisePositive);
    }

    public void configAbsoluteRange(boolean useZeroToThreeSixty)
    {
        this.wrappedObject.configAbsoluteSensorRange(useZeroToThreeSixty ? AbsoluteSensorRange.Unsigned_0_to_360 : AbsoluteSensorRange.Signed_PlusMinus180);
    }

    public void configMagnetOffset(double offsetDegrees)
    {
        this.wrappedObject.configMagnetOffset(offsetDegrees);
    }
}