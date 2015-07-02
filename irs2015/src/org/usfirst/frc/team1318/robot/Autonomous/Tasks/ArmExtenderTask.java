package org.usfirst.frc.team1318.robot.Autonomous.Tasks;

import org.usfirst.frc.team1318.robot.Autonomous.AutonomousControlData;
import org.usfirst.frc.team1318.robot.Autonomous.IAutonomousTask;

/**
 * ArmTiltTask:
 * 
 * This task extends or unextends the arm, sleeping for a certain (provided) length of time.
 * 
 * @author Will
 *
 */
public class ArmExtenderTask extends TimedAutonomousTask implements IAutonomousTask
{
    private final boolean extend;

    /**
     * Initializes a new ArmExtendTask
     * @param duration to perform the task in seconds
     * @param extend the arm (true) or retract the arm (false)
     */
    public ArmExtenderTask(double duration, boolean extend)
    {
        super(duration);

        this.extend = extend;
    }

    /**
     * Run an iteration of the current task and apply any control changes 
     * @param data to which we should apply updated settings
     */
    @Override
    public void update(AutonomousControlData data)
    {
        data.setArmExtenderExtendOverrideState(this.extend);
        data.setArmExtenderRetractOverrideState(!this.extend);
    }

    /**
     * Cancel the current task and clear control changes
     * @param data to which we should clear any updated control settings
     */
    @Override
    public void cancel(AutonomousControlData data)
    {
        super.cancel(data);

        data.setArmExtenderExtendOverrideState(false);
        data.setArmExtenderRetractOverrideState(false);
    }

    /**
     * End the current task and reset control changes appropriately
     * @param data to which we should apply updated settings
     */
    @Override
    public void end(AutonomousControlData data)
    {
        super.end(data);

        data.setArmExtenderExtendOverrideState(false);
        data.setArmExtenderRetractOverrideState(false);
    }
}
