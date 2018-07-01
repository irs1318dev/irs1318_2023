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
                double leftTicks = arcLength / HardwareConstants.DRIVETRAIN_LEFT_PULSE_DISTANCE;

                if (percentage <= 0.9)
                {
                    return percentage * leftTicks / 0.9;
                }

                return leftTicks;
            },
            percentage ->
            {
                double arcLength = Math.PI * HardwareConstants.DRIVETRAIN_WHEEL_SEPARATION_DISTANCE * (degrees / 360.0);
                double rightTicks = -arcLength / HardwareConstants.DRIVETRAIN_RIGHT_PULSE_DISTANCE;
                if (percentage <= 0.9)
                {
                    return percentage * rightTicks / 0.9;
                }

                return rightTicks;
            },
            duration);
    }
}
