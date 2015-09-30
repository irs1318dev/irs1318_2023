package org.usfirst.frc.team1318.robot.Driver.Autonomous.Tasks;

import org.usfirst.frc.team1318.robot.HardwareConstants;
import org.usfirst.frc.team1318.robot.DriveTrain.IDriveTrainComponent;
import org.usfirst.frc.team1318.robot.Driver.IControlTask;

/**
 * Autonomous task that turns the robot a certain amount clockwise or counterclockwise in-place using Positional PID.
 * 
 */
public class TurnAutonomousTask extends MoveDistanceAutonomousTaskBase implements IControlTask
{
    private final double degrees;

    /**
     * Initializes a new TurnAutonomousTask
     * @param degrees from the current orientation to rotate (positive means turn right/clockwise, negative means turn left/counter-clockwise)
     * @param driveTrain component to use to detect our current position
     */
    public TurnAutonomousTask(double degrees, IDriveTrainComponent driveTrain)
    {
        super(driveTrain);

        this.degrees = degrees;
    }

    /**
     * Determine the final encoder distance
     */
    @Override
    protected void determineFinalEncoderDistance()
    {
        double arcLength = Math.PI * HardwareConstants.DRIVETRAIN_WHEEL_SEPARATION_DISTANCE * (this.degrees / 360.0);
        this.desiredFinalLeftEncoderDistance = this.startLeftEncoderDistance + arcLength;
        this.desiredFinalRightEncoderDistance = this.startRightEncoderDistance - arcLength;
    }
}
