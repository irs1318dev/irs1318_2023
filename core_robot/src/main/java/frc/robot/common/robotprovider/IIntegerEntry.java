package frc.robot.common.robotprovider;

public interface IIntegerEntry
{
    long get();
    long get(long defaultValue);
    void set(long value);
    void setDefault(long defaultValue);
}
