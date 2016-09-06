package org.usfirst.frc.team1318.robot.Driver.ControlTasks;

import org.usfirst.frc.team1318.robot.DriveTrain.DriveTrainComponent;
import org.usfirst.frc.team1318.robot.Driver.IControlTask;
import org.usfirst.frc.team1318.robot.Driver.Operation;

public class PIDBrakeTask extends ControlTaskBase implements IControlTask
{
    public PIDBrakeTask()
    {
    }

    @Override
    public void begin()
    {
        DriveTrainComponent driveTrain = this.getComponents().getDriveTrain();
        this.setDigitalOperationState(Operation.DriveTrainUsePositionalMode, true);
        this.setAnalogOperationState(Operation.DriveTrainLeftPosition, driveTrain.getLeftEncoderDistance());
        this.setAnalogOperationState(Operation.DriveTrainRightPosition, driveTrain.getRightEncoderDistance());
    }

    @Override
    public void update()
    {
    }

    @Override
    public void stop()
    {
        this.setDigitalOperationState(Operation.DriveTrainUsePositionalMode, false);
        this.setAnalogOperationState(Operation.DriveTrainLeftPosition, 0.0);
        this.setAnalogOperationState(Operation.DriveTrainRightPosition, 0.0);
    }

    @Override
    public void end()
    {
        this.setDigitalOperationState(Operation.DriveTrainUsePositionalMode, false);
        this.setAnalogOperationState(Operation.DriveTrainLeftPosition, 0.0);
        this.setAnalogOperationState(Operation.DriveTrainRightPosition, 0.0);
    }

    @Override
    public boolean shouldCancel()
    {
        return false;
    }

    @Override
    public boolean hasCompleted()
    {
        return false;
    }
}
