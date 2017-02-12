package org.usfirst.frc.team1318.robot.common;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Logger that logs current values to a dashboard.
 *
 */
public class ConsoleDashboardLogger implements IDashboardLogger
{
    /**
     * Write a boolean to the smart dashboard
     * @param component to log for
     * @param key to write to
     * @param value to write
     */
    public void logBoolean(String component, String key, boolean value)
    {
        String logKey = String.format("%s.%s", component, key);
        System.out.println(logKey + ": " + value);
    }

    /**
     * Write a number (double) to the smart dashboard
     * @param component to log for
     * @param key to write to
     * @param value to write
     */
    public void logNumber(String component, String key, double value)
    {
        String logKey = String.format("%s.%s", component, key);
        System.out.println(logKey + ": " + value);
    }

    /**
     * Write a number (double) to the smart dashboard
     * @param component to log for
     * @param key to write to
     * @param value to write
     */
    public void logNumber(String component, String key, Double value)
    {
        String logKey = String.format("%s.%s", component, key);
        String valueString = "N/A";
        if (value != null)
        {
            valueString = "" + value;
        }

        SmartDashboard.putString(logKey, valueString);
    }

    /**
     * Write a number (integer) to the smart dashboard
     * @param component to log for
     * @param key to write to
     * @param value to write
     */
    public void logInteger(String component, String key, int value)
    {
        this.logInteger(component, key, value, null);
    }

    /**
     * Write a number (integer) to the smart dashboard
     * @param component to log for
     * @param key to write to
     * @param value to write
     * @param formatString to use
     */
    public void logInteger(String component, String key, int value, String formatString)
    {
        String logKey = String.format("%s.%s", component, key);

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

    /**
     * Write a string to the smart dashboard
     * @param component to log for
     * @param key to write to
     * @param value to write
     */
    public void logString(String component, String key, String value)
    {
        String logKey = String.format("%s.%s", component, key);
        System.out.println(logKey + ": " + value);
    }
}
