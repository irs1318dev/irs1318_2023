package frc.robot.driver.controltasks;

import frc.robot.driver.*;

public class DriveSimplePathTask extends TimedTask
{
    private double velocityL;
    private double velocityR;

    public DriveSimplePathTask(double duration, double velocityL, double velocityR)
    {
        super(duration);

        this.velocityL = velocityL;
        this.velocityR = velocityR;
    }

    @Override
    public void update()
    {
        this.setDigitalOperationState(DigitalOperation.DriveTrainUseSimplePathMode, true);
        this.setAnalogOperationState(AnalogOperation.DriveTrainLeftVelocity, this.velocityL);
        this.setAnalogOperationState(AnalogOperation.DriveTrainRightVelocity, this.velocityR);
    }
}
