package org.usfirst.frc.team1318.robot.driver.controltasks;

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
                if (percentage <= 0.9)
                {
                    return percentage * distance / 0.9;
                }

                return distance;
            },
            percentage ->
            {
                if (percentage <= 0.9)
                {
                    return percentage * distance / 0.9;
                }

                return distance;
            },
            duration);
    }
}
