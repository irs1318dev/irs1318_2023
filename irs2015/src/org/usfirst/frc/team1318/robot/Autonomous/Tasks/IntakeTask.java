package org.usfirst.frc.team1318.robot.Autonomous.Tasks;

import org.usfirst.frc.team1318.robot.Autonomous.AutonomousControlData;

/**
 * IntakeOutTask:
 * 
 * This task runs the intake in a certain direction for a certain period of time
 * 
 * @author Will
 *
 */
public class IntakeTask extends TimedAutonomousTask
{
    private final boolean out;

    /**
     * Initializes a new IntakeOutTask
     * @param duration to perform the task in seconds
     * @param out indicates whether the intake should be run "out" (true) or "in" (false) 
     */
    public IntakeTask(double duration, boolean out)
    {
        super(duration);

        this.out = out;
    }

    /**
     * Run an iteration of the current task and apply any control changes 
     * @param data to which we should apply updated settings
     */
    @Override
    public void update(AutonomousControlData data)
    {
        data.setIntakeBackwardState(this.out);
        data.setIntakeForwardState(!this.out);
    }

    /**
     * Cancel the current task and clear control changes
     * @param data to which we should clear any updated control settings
     */
    @Override
    public void cancel(AutonomousControlData data)
    {
        super.cancel(data);

        data.setIntakeBackwardState(false);
        data.setIntakeForwardState(false);
    }

    /**
     * End the current task and reset control changes appropriately
     * @param data to which we should apply updated settings
     */
    @Override
    public void end(AutonomousControlData data)
    {
        super.end(data);

        data.setIntakeBackwardState(false);
        data.setIntakeForwardState(false);
    }
}
