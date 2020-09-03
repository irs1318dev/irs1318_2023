package frc.robot.common.robotprovider;

import frc.robot.LoggingKey;

/**
 * Logger that logs current values to a dashboard.
 */
public class ConsoleDashboardLogger extends StringLogger
{
    /**
     * Write a string to the log
     * @param key to write to
     * @param value to write
     */
    @Override
    public void logString(LoggingKey key, String value)
    {
        System.out.println(key.value + ": " + value);
    }
}
