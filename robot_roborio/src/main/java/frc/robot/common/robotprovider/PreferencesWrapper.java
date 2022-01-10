package frc.robot.common.robotprovider;

import java.util.Collection;

import javax.inject.Singleton;

import edu.wpi.first.wpilibj.Preferences;

@Singleton
public class PreferencesWrapper implements IPreferences
{
    public PreferencesWrapper()
    {
    }

    /**
     * Returns whether or not there is a key with the given name.
     * */
    @Override
    public boolean containsKey(String key)
    {
        return Preferences.containsKey(key);
    }

    /**
     * Returns the boolean at the given key.
     * */
    @Override
    public boolean getBoolean(String key, boolean backup)
    {
        return Preferences.getBoolean(key, backup);
    }

    /**
     * Returns the double at the given key.
     * */
    @Override
    public double getDouble(String key, double backup)
    {
        return Preferences.getDouble(key, backup);
    }

    /**
     * Returns the float at the given key.
     * */
    @Override
    public float getFloat(String key, float backup)
    {
        return Preferences.getFloat(key, backup);
    }

    /**
     * Returns the int at the given key.
     * */
    @Override
    public int getInt(String key, int backup)
    {
        return Preferences.getInt(key, backup);
    }

    /**
     * Gets the preferences keys.
     * */
    @Override
    public Collection<String> getKeys()
    {
        return Preferences.getKeys();
    }

    /**
     * Returns the long at the given key.
     * */
    @Override
    public long getLong(String key, long backup)
    {
        return Preferences.getLong(key, backup);
    }

    /**
     * Returns the string at the given key.
     * */
    @Override
    public String getString(String key, String backup)
    {
        return Preferences.getString(key, backup);
    }

    /**
     * Puts the given boolean into the preferences table if it doesn't already exist.
     * */
    @Override
    public void initBoolean(String key, boolean value)
    {
        Preferences.initBoolean(key, value);
    }

    /**
     * Puts the given double into the preferences table if it doesn't already exist.
     * */
    @Override
    public void initDouble(String key, double value)
    {
        Preferences.initDouble(key, value);
    }

    /**
     * Puts the given float into the preferences table if it doesn't already exist.
     * */
    @Override
    public void initFloat(String key, float value)
    {
        Preferences.initFloat(key, value);
    }

    /**
     * Puts the given int into the preferences table if it doesn't already exist.
     * */
    @Override
    public void initInt(String key, int value)
    {
        Preferences.initInt(key, value);
    }

    /**
     * Puts the given long into the preferences table if it doesn't already exist.
     * */
    @Override
    public void initLong(String key, long value)
    {
        Preferences.initLong(key, value);
    }

    /**
     * Puts the given string into the preferences table if it doesn't already exist.
     * */
    @Override
    public void initString(String key, String value)
    {
        Preferences.initString(key, value);
    }

    /**
     * Puts the given boolean into the preferences table.
     * */
    @Override
    public void putBoolean(String key, boolean value)
    {
        Preferences.putBoolean(key, value);
    }

    /**
     * Puts the given double into the preferences table.
     * */
    @Override
    public void putDouble(String key, double value)
    {
        Preferences.putDouble(key, value);
    }

    /**
     * Puts the given float into the preferences table.
     * */
    @Override
    public void putFloat(String key, float value)
    {
        Preferences.putFloat(key, value);
    }

    /**
     * Puts the given int into the preferences table.
     * */
    @Override
    public void putInt(String key, int value)
    {
        Preferences.putInt(key, value);
    }

    /**
     * Puts the given long into the preferences table.
     * */
    @Override
    public void putLong(String key, long value)
    {
        Preferences.putLong(key, value);
    }

    /**
     * Puts the given string into the preferences table.
     * */
    @Override
    public void putString(String key, String value)
    {
        Preferences.putString(key, value);
    }

    /**
     * Remove a preference.
     * */
    @Override
    public void remove(String key)
    {
        Preferences.remove(key);
    }

    /**
     * Remove all preferences.
     * */
    @Override
    public void removeAll()
    {
        Preferences.removeAll();
    }
}