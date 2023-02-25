package frc.robot.driver.controltasks;

public class VisionMoveAndTurnTask extends VisionMoveAndTurnTaskBase
{
    private final double desiredDistance;

    /**
     * Initializes a new VisionMoveAndTurnTask
     * @param useFastMode whether to use faster PID settings to move towards the goal position, or regular settings
     * @param useSlowMode 
     * @param rotateType the type of rotation we are trying to perform (e.g. RetroReflective, AprilTag Center/Yaw, etc.)
     * @param translateType the type of translation we are trying to perform (e.g. Forward, Strafe)
     * @param bestEffort whether to consider the task successful/completed if we stop seeing the vision target instead of cancelled
     * @param verifyAngle whether to verify the angle when we declare completed as well as distance, or consider ourselves completed when we are at the goal distance
     * @param desiredDistance the desired distance value to reach
     */
    public VisionMoveAndTurnTask(boolean useFastMode, TurnType rotateType, MoveType translateType, boolean bestEffort, boolean verifyAngle, double desiredDistance)
    {
        super(useFastMode, rotateType, translateType, bestEffort, verifyAngle);
        this.desiredDistance = desiredDistance;
    }

    @Override
    protected double getDesiredDistance(double currentDistance)
    {
        return this.desiredDistance;
    }
}