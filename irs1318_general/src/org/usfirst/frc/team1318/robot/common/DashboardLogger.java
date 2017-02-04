package org.usfirst.frc.team1318.robot.common;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Logger that logs current values to a dashboard.
 *
 */
public class DashboardLogger
{
    private enum DashboardMode
    {
        SmartDashboard,
        Console;
    }

    /**
     * Default instance
     */
    private static DashboardMode defaultMode = DashboardMode.SmartDashboard;

    /**
     * Write a boolean to the smart dashboard
     * @param component to log for
     * @param key to write to
     * @param value to write
     */
    public static void logBoolean(String component, String key, boolean value)
    {
        String logKey = String.format("%s.%s", component, key);
        if (DashboardLogger.defaultMode == DashboardMode.SmartDashboard)
        {
            if (SmartDashboard.getBoolean(logKey, !value) != value)
            {
                SmartDashboard.putBoolean(logKey, value);
            }
        }
        else if (DashboardLogger.defaultMode == DashboardMode.Console)
        {
            System.out.println(logKey + ": " + value);
        }
        else
        {

        }
    }

    /**
     * Write a number (double) to the smart dashboard
     * @param component to log for
     * @param key to write to
     * @param value to write
     */
    public static void logNumber(String component, String key, double value)
    {
        String logKey = String.format("%s.%s", component, key);
        if (DashboardLogger.defaultMode == DashboardMode.SmartDashboard)
        {
            if (SmartDashboard.getNumber(logKey, value + 0.5) != value)
            {
                SmartDashboard.putNumber(logKey, value);
            }
        }
        else if (DashboardLogger.defaultMode == DashboardMode.Console)
        {
            System.out.println(logKey + ": " + value);
        }
        else
        {

        }
    }

    /**
     * Write a number (integer) to the smart dashboard
     * @param component to log for
     * @param key to write to
     * @param value to write
     */
    public static void logInteger(String component, String key, int value)
    {
        DashboardLogger.logInteger(component, key, value, null);
    }

    /**
     * Write a number (integer) to the smart dashboard
     * @param component to log for
     * @param key to write to
     * @param value to write
     * @param formatString to use
     */
    public static void logInteger(String component, String key, int value, String formatString)
    {
        String logKey = String.format("%s.%s", component, key);
        if (DashboardLogger.defaultMode == DashboardMode.SmartDashboard)
        {
            if (SmartDashboard.getNumber(logKey, value + 0.5) != value)
            {
                SmartDashboard.putNumber(logKey, value);
            }
        }
        else if (DashboardLogger.defaultMode == DashboardMode.Console)
        {
            String valueOutput;
            if (formatString != null && !formatString.isEmpty())
            {
                valueOutput = String.format(formatString, value);
            }
            else
            {
                valueOutput = Integer.toString(value);
            }

            System.out.println(logKey + ": " + valueOutput);
        }
        else
        {

        }
    }

    /**
     * Write a string to the smart dashboard
     * @param component to log for
     * @param key to write to
     * @param value to write
     */
    public static void logString(String component, String key, String value)
    {
        String logKey = String.format("%s.%s", component, key);
        if (DashboardLogger.defaultMode == DashboardMode.SmartDashboard)
        {
            if (SmartDashboard.getString(logKey, null) != value)
            {
                SmartDashboard.putString(logKey, value);
            }
        }
        else if (DashboardLogger.defaultMode == DashboardMode.Console)
        {
            System.out.println(logKey + ": " + value);
        }
        else
        {

        }
    }
}
