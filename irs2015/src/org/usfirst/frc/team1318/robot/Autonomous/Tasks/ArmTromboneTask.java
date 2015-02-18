package org.usfirst.frc.team1318.robot.Autonomous.Tasks;

import org.usfirst.frc.team1318.robot.Autonomous.AutonomousControlData;
import org.usfirst.frc.team1318.robot.Autonomous.IAutonomousTask;

/**
 * ArmTiltTask:
 * 
 * This task opens or closes the arm's trombone, sleeping for a certain (provided) length of time.
 * 
 * @author Will
 *
 */
public class ArmTromboneTask extends TimedAutonomousTask implements IAutonomousTask
{
    private final boolean open;

    /**
     * Initializes a new ArmTromboneTask
     * @param duration to perform the task in seconds
     * @param open the trobone (true) or close the trombone (false)
     */
    public ArmTromboneTask(double duration, boolean open)
    {
        super(duration);

        this.open = open;
    }

    /**
     * Run an iteration of the current task and apply any control changes 
     * @param data to which we should apply updated settings
     */
    @Override
    public void update(AutonomousControlData data)
    {
        data.setArmTromboneExtendOverrideState(this.open);
        data.setArmTromboneRetractOverrideState(!this.open);
    }

    /**
     * Cancel the current task and clear control changes
     * @param data to which we should clear any updated control settings
     */
    @Override
    public void cancel(AutonomousControlData data)
    {
        super.cancel(data);

        data.setArmTromboneExtendOverrideState(false);
        data.setArmTromboneRetractOverrideState(false);
    }

    /**
     * End the current task and reset control changes appropriately
     * @param data to which we should apply updated settings
     */
    @Override
    public void end(AutonomousControlData data)
    {
        super.end(data);

        data.setArmTromboneExtendOverrideState(false);
        data.setArmTromboneRetractOverrideState(false);
    }
}
