package org.usfirst.frc.team1318.robot.driver.controltasks;

import org.usfirst.frc.team1318.robot.HardwareConstants;

public class DriveDistanceTimedTask extends DriveRouteTask
{
    /**
     * Initializes a new DriveDistanceRouteTask
     * @param distance to travel overall
     * @param duration to take to drive the specified distance
     */
    public DriveDistanceTimedTask(double distance, double duration)
    {
        super(
            percentage ->
            {
                double ticks = distance / HardwareConstants.DRIVETRAIN_LEFT_PULSE_DISTANCE;
                if (percentage <= 0.9)
                {
                    return ticks * (percentage / 0.9);
                }

                return ticks;
            },
            percentage ->
            {
                double ticks = distance / HardwareConstants.DRIVETRAIN_RIGHT_PULSE_DISTANCE;
                if (percentage <= 0.9)
                {
                    return ticks * (percentage / 0.9);
                }

                return ticks;
            },
            duration);
    }
}
