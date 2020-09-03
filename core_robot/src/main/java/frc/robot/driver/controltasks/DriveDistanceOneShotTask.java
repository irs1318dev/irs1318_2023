package frc.robot.driver.controltasks;

/**
 * Task that drives the robot a certain distance directly forward or backward using Positional PID.
 * 
 */
public class DriveDistanceOneShotTask extends MoveDistanceOneShotTaskBase
{
    private final double distance;

    /**
     * Initializes a new DriveDistanceTask
     * @param distance from the current location to move (positive means move forward, negative means move backwards) in centimeters
     */
    public DriveDistanceOneShotTask(double distance)
    {
        this(distance, true);
    }

    /**
     * Initializes a new DriveDistanceTask
     * @param distance from the current location to move (positive means move forward, negative means move backwards) in centimeters
     * @param resetPositionalOnEnd
     */
    public DriveDistanceOneShotTask(double distance, boolean resetPositionalOnEnd)
    {
        super(resetPositionalOnEnd);

        this.distance = distance;
    }

    /**
     * Determine the final encoder distance
     */
    @Override
    protected void determineFinalEncoderDistance()
    {
        this.desiredFinalLeftTicks = this.startLeftTicks + this.distance;
        this.desiredFinalRightTicks = this.startRightTicks + this.distance;
    }
}
