package org.usfirst.frc.team1318.robot.Common;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.tables.TableKeyNotDefinedException;

/**
 * Logger that logs current values to the smart dashboard. 
 * 
 * @author Chris K
 *
 */
public class SmartDashboardLogger
{
    /**
     * Write a boolean to the smart dashboard
     * @param key to write to
     * @param value to write
     */
    public static void putBoolean(String key, boolean value)
    {
        SmartDashboardLogger.putBoolean(key, new Boolean(value));
    }

    /**
     * Write a Boolean to the smart dashboard
     * @param key to write to
     * @param value to write
     */
    public static void putBoolean(String key, Boolean value)
    {
        if (value == null)
        {
            return;
        }

        try
        {
            if (SmartDashboard.getBoolean(key) != value.booleanValue())
            {
                SmartDashboard.putBoolean(key, value.booleanValue());
            }
        }
        catch (TableKeyNotDefinedException ex)
        {
            SmartDashboard.putBoolean(key, value.booleanValue());
        }
    }

    /**
     * Write a number (double) to the smart dashboard
     * @param key to write to
     * @param value to write
     */
    public static void putNumber(String key, double value)
    {
        SmartDashboardLogger.putNumber(key, new Double(value));
    }

    /**
     * Write a number (Double) to the smart dashboard
     * @param key to write to
     * @param value to write
     */
    public static void putNumber(String key, Double value)
    {
        if (value == null)
        {
            return;
        }

        try
        {
            if (SmartDashboard.getNumber(key) != value.doubleValue())
            {
                SmartDashboard.putNumber(key, value.doubleValue());
            }
        }
        catch (TableKeyNotDefinedException ex)
        {
            SmartDashboard.putNumber(key, value.doubleValue());
        }
    }

    /**
     * Write a string to the smart dashboard
     * @param key to write to
     * @param value to write
     */
    public static void putString(String key, String value)
    {
        try
        {
            if (SmartDashboard.getString(key) != value)
            {
                SmartDashboard.putString(key, value);
            }
        }
        catch (TableKeyNotDefinedException ex)
        {
            SmartDashboard.putString(key, value);
        }
    }

    /**
     * Get a boolean from the smart dashboard
     * @param key to retrieve
     * @return value from smart dashboard
     */
    public static boolean getBoolean(String key)
    {
        return SmartDashboard.getBoolean(key);
    }

    /**
      * Get a number (double) from the smart dashboard
      * @param key to retrieve
      * @return value from smart dashboard
      */
    public static double getNumber(String key)
    {
        return SmartDashboard.getNumber(key);
    }

    /**
     * Get a string from the smart dashboard
     * @param key to retrieve
     * @return value from smart dashboard
     */
    public static String getString(String key)
    {
        return SmartDashboard.getString(key);
    }
}
