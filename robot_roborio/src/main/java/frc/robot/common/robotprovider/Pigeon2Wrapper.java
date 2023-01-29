package frc.robot.common.robotprovider;

import com.ctre.phoenix.sensors.*;

public class Pigeon2Wrapper implements IPigeon2
{
    private final Pigeon2 wrappedObject;

    public Pigeon2Wrapper(int deviceNumber)
    {
        this.wrappedObject = new Pigeon2(deviceNumber);
    }

    public Pigeon2Wrapper(int deviceNumber, String canbus)
    {
        this.wrappedObject = new Pigeon2(deviceNumber, canbus);
    }

    public void getYawPitchRoll(double[] ypr_deg)
    {
        CTREErrorCodeHelper.printError(
            this.wrappedObject.getYawPitchRoll(ypr_deg),
            "Pigeon2.getYawPitchRoll");
    }

    public void getRawGyro(double[] xyz_dps)
    {
        CTREErrorCodeHelper.printError(
            this.wrappedObject.getRawGyro(xyz_dps),
            "Pigeon2.getRawGyro");
    }

    public void setYaw(double angleDeg)
    {
        CTREErrorCodeHelper.printError(
            this.wrappedObject.setYaw(angleDeg),
            "Pigeon2.setYaw");
    }
}