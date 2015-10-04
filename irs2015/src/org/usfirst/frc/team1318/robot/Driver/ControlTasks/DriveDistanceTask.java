package org.usfirst.frc.team1318.robot.Driver.ControlTasks;

import org.usfirst.frc.team1318.robot.DriveTrain.DriveTrainComponent;
import org.usfirst.frc.team1318.robot.Driver.IControlTask;

/**
 * Task that drives the robot a certain distance directly forward or backward using Positional PID.
 * 
 */
public class DriveDistanceTask extends MoveDistanceTaskBase implements IControlTask
{
    private final double distance;

    /**
     * Initializes a new DriveDistanceTask
     * @param distance from the current location to move (positive means move forward, negative means move backwards) in centimeters
     * @param driveTrain component to use to detect our current position
     */
    public DriveDistanceTask(double distance, DriveTrainComponent driveTrain)
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
