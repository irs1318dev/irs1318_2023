package frc.robot.driver.controltasks;

import frc.robot.driver.Operation;

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
        this.setDigitalOperationState(Operation.DriveTrainUseSimplePathMode, true);
        this.setAnalogOperationState(Operation.DriveTrainLeftVelocity, this.velocityL);
        this.setAnalogOperationState(Operation.DriveTrainRightVelocity, this.velocityR);
    }
}
