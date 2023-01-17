package frc.robot.driver.controltasks;

import frc.robot.driver.*;
import frc.robot.mechanisms.DriveTrainMechanism;
import frc.robot.mechanisms.PigeonManager;

public class ChargePadLevellingTask extends ControlTaskBase
{
    private PigeonManager imuManager;
    private DriveTrainMechanism driveTrain;

    private double pitch;
    private double sign;
    private double xPos;

    public ChargePadLevellingTask()
    {
        this.pitch = 0.0;
        this.sign = 1.0;
        this.xPos = 1.0;
    }

    /**
     * Begin the current task.
     */
    @Override
    public void begin()
    {
        this.imuManager = this.getInjector().getInstance(PigeonManager.class);
        this.driveTrain = this.getInjector().getInstance(DriveTrainMechanism.class);

        this.setDigitalOperationState(DigitalOperation.DriveTrainEnableMaintainDirectionMode, true);
        this.setAnalogOperationState(AnalogOperation.DriveTrainTurnAngleGoal, 0.0);
        this.setAnalogOperationState(AnalogOperation.DriveTrainMoveRight, 0.0);
        this.setAnalogOperationState(AnalogOperation.DriveTrainMoveForward, 0.0);
    }

    /**
     * Run an iteration of the current task.
     */
    @Override
    public void update()
    {
        this.pitch = this.imuManager.getPitch();
        if (this.pitch > 0)
        {
            this.sign = 1;
        }
        else if (this.pitch < 0)
        {
            this.sign = -1;
        }

        this.xPos = this.driveTrain.getPositionX();
        this.setAnalogOperationState(AnalogOperation.DriveTrainMoveForward, this.sign * 2 / this.xPos);
    }

    /**
     * Ends the current task, called when it (or a master task) has completed.
     */
    @Override
    public void end()
    {
        this.setDigitalOperationState(DigitalOperation.DriveTrainEnableMaintainDirectionMode, false);
        this.setAnalogOperationState(AnalogOperation.DriveTrainMoveForward, 0.0);
    }

    /**
     * Checks whether this task has completed, or whether it should continue being processed.
     * @return true if we should continue onto the next task, otherwise false (to keep processing this task)
     */
    @Override
    public boolean hasCompleted()
    {
        return Math.abs(this.pitch) < 0.5;
    }
}
