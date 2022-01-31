package frc.robot.common.robotprovider;

import com.ctre.phoenix.sensors.*;

public class Pigeon2Wrapper implements IPigeon2
{
    private final Pigeon2 wrappedObject;

    public Pigeon2Wrapper(int deviceNumber)
    {
        this.wrappedObject = new Pigeon2(deviceNumber);
    }

    public int getYawPitchRoll(double[] ypr_deg)
    {
        return this.wrappedObject.getYawPitchRoll(ypr_deg).value;
    }

    public int setYaw(double angleDeg)
    {
        return this.wrappedObject.setYaw(angleDeg).value;
    }
}