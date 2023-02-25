package frc.robot.driver.controltasks;

import frc.robot.mechanisms.PigeonManager;

public class AntiTipTask extends ControlTaskBase
{
    private PigeonManager imuManager;

    public AntiTipTask()
    {
    }

    /**
     * Begin the current task.
     */
    @Override
    public void begin()
    {
        this.imuManager = this.getInjector().getInstance(PigeonManager.class);
    }

    /**
     * Run an iteration of the current task.
     */
    @Override
    public void update()
    {
        double pitch = this.imuManager.getPitch();
        double roll = this.imuManager.getRoll();
    }

    /**
     * Ends the current task, called when it (or a master task) has completed.
     */
    @Override
    public void end()
    {
    }

    /**
     * Checks whether this task has completed, or whether it should continue being processed.
     * @return true if we should continue onto the next task, otherwise false (to keep processing this task)
     */
    @Override
    public boolean hasCompleted()
    {
        return true;
    }
}
