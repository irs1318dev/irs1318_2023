package org.usfirst.frc.team1318.robot.driver.controltasks;

import org.usfirst.frc.team1318.robot.TuningConstants;
import org.usfirst.frc.team1318.robot.common.PIDHandler;
import org.usfirst.frc.team1318.robot.driver.IControlTask;
import org.usfirst.frc.team1318.robot.driver.Operation;
import org.usfirst.frc.team1318.robot.vision.VisionManager;

/**
 * Task that turns the robot a certain amount clockwise or counterclockwise in-place based on vision center
 * 
 * @author William
 */
public class VisionCenteringTask extends ControlTaskBase implements IControlTask
{
    private final PIDHandler turnPidHandler;
    protected VisionManager visionManager;

    /**
    * Initializes a new VisionCenteringTask
    */
    public VisionCenteringTask()
    {
        this.turnPidHandler = new PIDHandler(0.15, 0.0, 0.0, 0.0, -0.3, 0.3);
    }

    /**
     * Begin the current task
     */
    @Override
    public void begin()
    {
        this.visionManager = this.getInjector().getInstance(VisionManager.class);
    }

    /**
     * Run an iteration of the current task and apply any control changes
     */
    @Override
    public void update()
    {
        this.setDigitalOperationState(Operation.DriveTrainUsePositionalMode, false);

        Double currentMeasuredAngle = this.visionManager.getMeasuredAngle();
        Double currentDesiredAngle = this.visionManager.getDesiredAngle();
        if (currentMeasuredAngle != null && currentDesiredAngle != null)
        {
            this.setAnalogOperationState(
                Operation.DriveTrainTurn,
                -this.turnPidHandler.calculatePosition(currentDesiredAngle, currentMeasuredAngle));
        }
    }

    /**
     * Cancel the current task and clear control changes
     */
    @Override
    public void stop()
    {
        this.setDigitalOperationState(Operation.DriveTrainUsePositionalMode, false);
        this.setAnalogOperationState(Operation.DriveTrainTurn, 0.0);
    }

    /**
     * End the current task and reset control changes appropriately
     */
    @Override
    public void end()
    {
        this.setDigitalOperationState(Operation.DriveTrainUsePositionalMode, false);
        this.setAnalogOperationState(Operation.DriveTrainTurn, 0.0);
    }

    /**
     * Checks whether this task has completed, or whether it should continue being processed
     * @return true if we should continue onto the next task, otherwise false (to keep processing this task)
     */
    @Override
    public boolean hasCompleted()
    {
        Double currentMeasuredAngle = this.visionManager.getMeasuredAngle();
        Double currentDesiredAngle = this.visionManager.getDesiredAngle();
        if (currentMeasuredAngle == null || currentDesiredAngle == null)
        {
            return false;
        }

        double centerAngleDifference = Math.abs(currentMeasuredAngle - currentDesiredAngle);
        double output = this.turnPidHandler.getCurrentOutput();
        return centerAngleDifference < TuningConstants.MAX_VISION_CENTERING_RANGE_DEGREES &&
            Math.abs(output) < TuningConstants.MAX_VISION_CENTERING_OUTPUT;
    }

    @Override
    public boolean shouldCancel()
    {
        return this.visionManager.getCenter() == null;
    }
}
