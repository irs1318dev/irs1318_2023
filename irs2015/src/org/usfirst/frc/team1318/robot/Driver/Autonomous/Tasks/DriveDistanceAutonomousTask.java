package org.usfirst.frc.team1318.robot.Driver.Autonomous.Tasks;

import org.usfirst.frc.team1318.robot.DriveTrain.IDriveTrainComponent;
import org.usfirst.frc.team1318.robot.Driver.IControlTask;

/**
 * Autonomous task that drives the robot a certain distance directly forward or backward using Positional PID.
 * 
 */
public class DriveDistanceAutonomousTask extends MoveDistanceAutonomousTaskBase implements IControlTask
{
    private final double distance;

    /**
     * Initializes a new DriveDistanceAutonomousTask
     * @param distance from the current location to move (positive means move forward, negative means move backwards) in centimeters
     * @param driveTrain component to use to detect our current position
     */
    public DriveDistanceAutonomousTask(double distance, IDriveTrainComponent driveTrain)
    {
        super(driveTrain);

        this.distance = distance;
    }

    /**
     * Determine the final encoder distance
     */
    @Override
    protected void determineFinalEncoderDistance()
    {
        this.desiredFinalLeftEncoderDistance = this.startLeftEncoderDistance + this.distance;
        this.desiredFinalRightEncoderDistance = this.startRightEncoderDistance + this.distance;
    }
}
