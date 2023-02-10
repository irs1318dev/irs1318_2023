package frc.robot.mechanisms;

/**
 * Basic structure to hold a angle pair for the robot arm
 */
class ArmAngleSetpoint
{
    public final double upperAngle;
    public final double lowerAngle;

    public ArmAngleSetpoint(double upperAngle, double lowerAngle)
    {
        this.upperAngle = upperAngle;
        this.lowerAngle = lowerAngle;
    }
}