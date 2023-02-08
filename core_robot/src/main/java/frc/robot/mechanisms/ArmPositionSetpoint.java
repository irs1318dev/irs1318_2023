package frc.robot.mechanisms;

/**
 * Basic structure to hold a position pair for the robot arm
 */
class ArmPositionSetpoint
{
    public final double upperPosition;
    public final double lowerPosition;

    public ArmPositionSetpoint(double upperPosition, double lowerPosition)
    {
        this.upperPosition = upperPosition;
        this.lowerPosition = lowerPosition;
    }
}