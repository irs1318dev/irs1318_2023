package org.usfirst.frc.team1318.robot.Autonomous.Tasks;

import org.usfirst.frc.team1318.robot.Autonomous.IAutonomousTask;
import org.usfirst.frc.team1318.robot.DriveTrain.IDriveTrainComponent;

/**
 * Autonomous task that drives the robot a certain distance directly forward or backward using Positional PID.
 * 
 * @author Will
 *
 */
public class DriveDistanceAutonomousTask extends MoveDistanceAutonomousTaskBase implements IAutonomousTask
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
    protected void determineFinalEncoderDistance()
    {
        this.desiredFinalLeftEncoderDistance = this.startLeftEncoderDistance + this.distance;
        this.desiredFinalRightEncoderDistance = this.startRightEncoderDistance + this.distance;
    }
}
