package org.usfirst.frc.team1318.robot.common;

public abstract class StringLogger implements IDashboardLogger
{
    /**
     * Write a string to the smart dashboard
     * @param component to log for
     * @param key to write to
     * @param value to write
     */
    public abstract void logString(String component, String key, String value);

    /**
     * Write a boolean to the smart dashboard
     * @param component to log for
     * @param key to write to
     * @param value to write
     */
    @Override
    public void logBoolean(String component, String key, boolean value)
    {
        this.logString(component, key, String.valueOf(value));
    }

    /**
     * Write a number (double) to the smart dashboard
     * @param component to log for
     * @param key to write to
     * @param value to write
     */
    @Override
    public void logNumber(String component, String key, double value)
    {
        this.logString(component, key, String.valueOf(value));
    }

    /**
     * Write a number (double) to the smart dashboard
     * @param component to log for
     * @param key to write to
     * @param value to write
     */
    @Override
    public void logNumber(String component, String key, Double value)
    {
        this.logString(component, key, String.valueOf(value));
    }

    /**
     * Write a number (integer) to the smart dashboard
     * @param component to log for
     * @param key to write to
     * @param value to write
     */
    @Override
    public void logInteger(String component, String key, int value)
    {
        this.logString(component, key, String.valueOf(value));
    }

    /**
     * Write a number (integer) to the smart dashboard
     * @param component to log for
     * @param key to write to
     * @param value to write
     * @param formatString to use
     */
    @Override
    public void logInteger(String component, String key, int value, String formatString)
    {
        this.logString(component, key, String.format(formatString, value));
    }

    /**
     * Flush the output stream, if appropriate..
     */
    @Override
    public void flush()
    {
    }
}
