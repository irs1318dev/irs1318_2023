package org.usfirst.frc.team1318.robot.Autonomous.Tasks;

import org.usfirst.frc.team1318.robot.Autonomous.AutonomousConstants;
import org.usfirst.frc.team1318.robot.Autonomous.AutonomousControlData;
import org.usfirst.frc.team1318.robot.Autonomous.IAutonomousTask;
import org.usfirst.frc.team1318.robot.DriveTrain.IDriveTrainComponent;

/**
 * Abstract class defining a task that moves the robot a certain distance using the drive train.
 * 
 * @author Will
 *
 */
public abstract class MoveDistanceAutonomousTaskBase implements IAutonomousTask
{
    private final IDriveTrainComponent driveTrain;

    protected double startLeftEncoderDistance;
    protected double startRightEncoderDistance;

    protected double desiredFinalLeftEncoderDistance;
    protected double desiredFinalRightEncoderDistance;

    /**
     * Initializes a new MoveDistanceAutonomousTaskBase
     * @param driveTrain component to use to detect our current position
     */
    protected MoveDistanceAutonomousTaskBase(IDriveTrainComponent driveTrain)
    {
        this.driveTrain = driveTrain;
    }

    /**
     * Begin the current task
     */
    public void begin()
    {
        // get the start location
        this.startLeftEncoderDistance = this.driveTrain.getLeftEncoderDistance();
        this.startRightEncoderDistance = this.driveTrain.getRightEncoderDistance();

        // calculate the desired end location
        this.determineFinalEncoderDistance();
    }

    /**
     * Determine the final encoder distance
     */
    protected abstract void determineFinalEncoderDistance();

    /**
     * Run an iteration of the current task and apply any control changes 
     * @param data to which we should apply updated settings
     */
    public void update(AutonomousControlData data)
    {
        data.setDriveTrainLeftPosition(this.desiredFinalLeftEncoderDistance);
        data.setDriveTrainRightPosition(this.desiredFinalRightEncoderDistance);
        data.setDriveTrainPositionMode(true);
    }

    /**
     * Cancel the current task and clear control changes
     * @param data to which we should clear any updated control settings
     */
    public void cancel(AutonomousControlData data)
    {
        data.setDriveTrainLeftPosition(0.0);
        data.setDriveTrainRightPosition(0.0);
        data.setDriveTrainPositionMode(false);
    }

    /**
     * End the current task and reset control changes appropriately
     * @param data to which we should apply updated settings
     */
    public void end(AutonomousControlData data)
    {
        data.setDriveTrainLeftPosition(0.0);
        data.setDriveTrainRightPosition(0.0);
        data.setDriveTrainPositionMode(false);
    }

    /**
     * Checks whether we should continue processing this task or whether it should end
     * @return true if we should continue on the current task, otherwise false (to move to the next task)
     */
    public boolean shouldContinue()
    {
        double leftEncoderDistance = this.driveTrain.getLeftEncoderDistance();
        double rightEncoderDistance = this.driveTrain.getRightEncoderDistance();

        // check how far away we are from the desired end location
        double leftDelta = Math.abs(this.desiredFinalLeftEncoderDistance - leftEncoderDistance);
        double rightDelta = Math.abs(this.desiredFinalRightEncoderDistance - rightEncoderDistance);

        // return that we should continue processing this task if we not are within an acceptable distance
        // from the desired end location for both left and right. 
        return leftDelta >= AutonomousConstants.DRIVETRAIN_POSITIONAL_ACCEPTABLE_DELTA &&
            rightDelta >= AutonomousConstants.DRIVETRAIN_POSITIONAL_ACCEPTABLE_DELTA;
    }
}
