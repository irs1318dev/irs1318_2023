package frc.robot.driver.controltasks;


public class VisionAdvanceAndCenterTask extends VisionAdvanceAndCenterTaskBase
{
    /**
    * Initializes a new VisionForwardAndCenterTask
    */
    public VisionAdvanceAndCenterTask(boolean useFastMode, boolean gamePiece, boolean bestEffort)
    {
        super(useFastMode, gamePiece, bestEffort, false);
    }

    @Override
    protected double getDesiredDistance(double currentDistance)
    {
        return 0.0;
    }
}