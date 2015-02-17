package org.usfirst.frc.team1318.robot.Autonomous.Tasks;

import org.usfirst.frc.team1318.robot.Autonomous.AutonomousControlData;

/**
 * IntakeOutTask:
 * 
 * This task runs the intake out for a certain period of time
 * 
 * @author Will
 *
 */
public class IntakeOutTask extends TimedAutonomousTask
{
    /**
     * Initializes a new IntakeOutTask
     * @param duration to perform the task in seconds
     */
    public IntakeOutTask(double duration)
    {
        super(duration);
    }

    /**
     * Run an iteration of the current task and apply any control changes 
     * @param data to which we should apply updated settings
     */
    @Override
    public void update(AutonomousControlData data)
    {
        data.setIntakeBackwardState(true);
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
    }
}
