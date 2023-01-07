package frc.robot.common.robotprovider;

import edu.wpi.first.networktables.GenericEntry;

public class GenericEntryWrapper implements IGenericEntry
{
    final GenericEntry wrappedObject;

    GenericEntryWrapper(GenericEntry object)
    {
        this.wrappedObject = object;
    }

    public double getDouble(double defaultValue)
    {
        return this.wrappedObject.getDouble(defaultValue);
    }

    public double[] getDoubleArray(double[] defaultValue)
    {
        return this.wrappedObject.getDoubleArray(defaultValue);
    }

    public boolean getBoolean(boolean defaultValue)
    {
        return this.wrappedObject.getBoolean(defaultValue);
    }

    public boolean[] getBooleanArray(boolean[] defaultValue)
    {
        return this.wrappedObject.getBooleanArray(defaultValue);
    }

    public long getInteger(long defaultValue)
    {
        return this.wrappedObject.getInteger(defaultValue);
    }

    public long[] getIntegerArray(long[] defaultValue)
    {
        return this.wrappedObject.getIntegerArray(defaultValue);
    }

    public String getString(String defaultValue)
    {
        return this.wrappedObject.getString(defaultValue);
    }

    public String[] getStringArray(String[] defaultValue)
    {
        return this.wrappedObject.getStringArray(defaultValue);
    }

    public void setDouble(double value)
    {
        this.wrappedObject.setDouble(value);
    }

    public void setDoubleArray(double[] value)
    {
        this.wrappedObject.setDoubleArray(value);
    }

    public void setBoolean(boolean value)
    {
        this.wrappedObject.setBoolean(value);
    }

    public void setBooleanArray(boolean[] value)
    {
        this.wrappedObject.setBooleanArray(value);
    }

    public void setInteger(long value)
    {
        this.wrappedObject.setInteger(value);
    }

    public void setIntegerArray(long[] value)
    {
        this.wrappedObject.setIntegerArray(value);
    }

    public void setString(String value)
    {
        this.wrappedObject.setString(value);
    }

    public void setStringArray(String[] value)
    {
        this.wrappedObject.setStringArray(value);
    }

    public void setDefaultDouble(double defaultValue)
    {
        this.wrappedObject.setDefaultDouble(defaultValue);
    }

    public void setDefaultDoubleArray(double[] defaultValue)
    {
        this.wrappedObject.setDefaultDoubleArray(defaultValue);
    }

    public void setDefaultBoolean(boolean defaultValue)
    {
        this.wrappedObject.setDefaultBoolean(defaultValue);
    }

    public void setDefaultBooleanArray(boolean[] defaultValue)
    {
        this.wrappedObject.setDefaultBooleanArray(defaultValue);
    }

    public void setDefaultInteger(long defaultValue)
    {
        this.wrappedObject.setDefaultInteger(defaultValue);
    }

    public void setDefaultIntegerArray(long[] defaultValue)
    {
        this.wrappedObject.setDefaultIntegerArray(defaultValue);
    }

    public void setDefaultString(String defaultValue)
    {
        this.wrappedObject.setDefaultString(defaultValue);
    }

    public void setDefaultStringArray(String[] defaultValue)
    {
        this.wrappedObject.setDefaultStringArray(defaultValue);
    }
}