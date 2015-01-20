package org.usfirst.frc.team1318.robot.Autonomous;

/**
 * Interface describing a task that is performed in autonomous mode.  The autonomous
 * task takes in the current control data as used by the autonomous driver and modifies
 * it.
 * 
 * @author Will
 *
 */
public interface IAutonomousTask
{
    /**
     * Begin the current task
     */
    public void begin();

    /**
     * Run an iteration of the current task and apply any control changes 
     * @param data to which we should apply updated settings
     */
    public void update(AutonomousControlData data);

    /**
     * Cancel the current task and clear control changes
     * @param data to which we should clear any updated control settings
     */
    public void cancel(AutonomousControlData data);

    /**
     * End the current task and reset control changes appropriately
     * @param data to which we should apply updated settings
     */
    public void end(AutonomousControlData data);

    /**
     * Checks whether we should continue processing this task or whether it should end
     * @return true if we should continue on the current task, otherwise false (to move to the next task)
     */
    public boolean shouldContinue();
}
