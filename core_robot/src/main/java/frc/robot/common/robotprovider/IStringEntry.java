package frc.robot.common.robotprovider;

public interface IStringEntry
{
    String get();
    String get(String defaultValue);
    void set(String value);
    void setDefault(String defaultValue);
}
