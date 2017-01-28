package org.usfirst.frc.team1318.robot.common.wpilibmocks;

public interface ICANTalon
{
    void set(double value);
    void changeControlMode(CANTalonControlMode mode);
    void setPIDF(double p, double i, double d, double f);
    void setPIDF(double p, double i, double d, double f, int izone, double closeLoopRampRate, int profile);
    void reverseOutput(boolean flip);
    void reverseSensor(boolean flip);
}
