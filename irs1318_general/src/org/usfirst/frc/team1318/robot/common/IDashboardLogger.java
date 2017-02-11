package org.usfirst.frc.team1318.robot.common;

public interface IDashboardLogger
{
    /**
     * Write a boolean to the smart dashboard
     * @param component to log for
     * @param key to write to
     * @param value to write
     */
    void logBoolean(String component, String key, boolean value);

    /**
     * Write a number (double) to the smart dashboard
     * @param component to log for
     * @param key to write to
     * @param value to write
     */
    void logNumber(String component, String key, double value);

    /**
     * Write a number (integer) to the smart dashboard
     * @param component to log for
     * @param key to write to
     * @param value to write
     */
    void logInteger(String component, String key, int value);

    /**
     * Write a number (integer) to the smart dashboard
     * @param component to log for
     * @param key to write to
     * @param value to write
     * @param formatString to use
     */
    void logInteger(String component, String key, int value, String formatString);

    /**
     * Write a string to the smart dashboard
     * @param component to log for
     * @param key to write to
     * @param value to write
     */
    void logString(String component, String key, String value);
}
