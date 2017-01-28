package org.usfirst.frc.team1318.robot.driver.controltasks;

import org.usfirst.frc.team1318.robot.HardwareConstants;
import org.usfirst.frc.team1318.robot.driver.IControlTask;
import org.usfirst.frc.team1318.robot.vision.VisionManager;

/**
 * Task that turns the robot a certain amount clockwise or counterclockwise in-place using Positional PID based on vision center
 * 
 * @author William
 */
public class VisionCenteringTask extends MoveDistanceTaskBase implements IControlTask
{
    private Double centerDegrees;

    /**
    * Initializes a new VisionCenteringTask
    */
    public VisionCenteringTask()
    {
        this(true);
    }

    /**
    * Initializes a new VisionCenteringTask
    */
    public VisionCenteringTask(boolean resetPositionOnEnd)
    {
        super(resetPositionOnEnd);
    }

    /**
     * Determine the final encoder distance based off center in pixels
     */
    @Override
    protected void determineFinalEncoderDistance()
    {
        // Convert center in pixels to degrees with 0 degrees being desired place
        Double centerAngle = this.getInjector().getInstance(VisionManager.class).getCenter1Angle();
        if (centerAngle != null)
        {
            // Set desired encoder distances based on degrees off of center
            double arcLength = Math.PI * HardwareConstants.DRIVETRAIN_WHEEL_SEPARATION_DISTANCE * (this.centerDegrees / 360.0);
            this.desiredFinalLeftEncoderDistance = this.startLeftEncoderDistance + arcLength;
            this.desiredFinalRightEncoderDistance = this.startLeftEncoderDistance - arcLength;
        }
        else
        {
            this.desiredFinalLeftEncoderDistance = this.startLeftEncoderDistance;
            this.desiredFinalRightEncoderDistance = this.startLeftEncoderDistance;
        }
    }
}
