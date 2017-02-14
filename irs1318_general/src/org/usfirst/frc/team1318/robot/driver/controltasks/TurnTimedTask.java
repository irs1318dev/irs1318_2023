package org.usfirst.frc.team1318.robot.driver.controltasks;

import org.usfirst.frc.team1318.robot.HardwareConstants;

public class TurnTimedTask extends DriveRouteTask
{
    /**
     * Initializes a new TurnRouteTask
     * @param degrees from the current orientation to rotate (positive means turn right/clockwise, negative means turn left/counter-clockwise)
     * @param duration to take to turn
     */
    public TurnTimedTask(double degrees, double duration)
    {
        super(
            percentage ->
            {
                double arcLength = Math.PI * HardwareConstants.DRIVETRAIN_WHEEL_SEPARATION_DISTANCE * (degrees / 360.0);
                double leftDistance = arcLength / 2.0;
                if (percentage <= 0.9)
                {
                    return percentage * leftDistance / 0.9;
                }

                return leftDistance;
            },
            percentage ->
            {
                double arcLength = Math.PI * HardwareConstants.DRIVETRAIN_WHEEL_SEPARATION_DISTANCE * (degrees / 360.0);
                double rightDistance = -arcLength / 2.0;
                if (percentage <= 0.9)
                {
                    return percentage * rightDistance / 0.9;
                }

                return rightDistance;
            },
            duration);
    }
}
