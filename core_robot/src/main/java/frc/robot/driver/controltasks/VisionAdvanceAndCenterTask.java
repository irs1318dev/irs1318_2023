package frc.robot.driver.controltasks;


public class VisionAdvanceAndCenterTask extends VisionAdvanceAndCenterTaskBase
{
    /**
    * Initializes a new VisionForwardAndCenterTask
    */
    public VisionAdvanceAndCenterTask(boolean useFastMode, boolean aprilTag, boolean bestEffort)
    {
        super(useFastMode, aprilTag, bestEffort, false);
    }

    @Override
    protected double getDesiredDistance(double currentDistance)
    {
        return 0.0;
    }
}