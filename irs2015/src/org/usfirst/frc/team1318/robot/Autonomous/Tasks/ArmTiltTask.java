package org.usfirst.frc.team1318.robot.Autonomous.Tasks;

import org.usfirst.frc.team1318.robot.Autonomous.AutonomousControlData;
import org.usfirst.frc.team1318.robot.Autonomous.IAutonomousTask;

/**
 * ArmTiltTask:
 * 
 * This task tilts or untilts the arm, sleeping for a certain (provided) length of time.
 * 
 * @author Will
 *
 */
public class ArmTiltTask extends TimedAutonomousTask implements IAutonomousTask
{
    private final boolean tilt;

    /**
     * Initializes a new ArmTiltTask
     * @param duration to perform the task in seconds
     * @param tilt the arm (true) or retract the arm (false)
     */
    public ArmTiltTask(double duration, boolean tilt)
    {
        super(duration);

        this.tilt = tilt;
    }

    /**
     * Run an iteration of the current task and apply any control changes 
     * @param data to which we should apply updated settings
     */
    @Override
    public void update(AutonomousControlData data)
    {
        data.setArmTiltExtendOverrideState(!this.tilt);
        data.setArmTiltRetractOverrideState(this.tilt);
    }

    /**
     * Cancel the current task and clear control changes
     * @param data to which we should clear any updated control settings
     */
    @Override
    public void cancel(AutonomousControlData data)
    {
        super.cancel(data);

        data.setArmTiltExtendOverrideState(false);
        data.setArmTiltRetractOverrideState(false);
    }

    /**
     * End the current task and reset control changes appropriately
     * @param data to which we should apply updated settings
     */
    @Override
    public void end(AutonomousControlData data)
    {
        super.end(data);

        data.setArmTiltExtendOverrideState(false);
        data.setArmTiltRetractOverrideState(false);
    }
}
