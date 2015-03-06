package org.usfirst.frc.team1318.robot.Autonomous.Tasks;

import org.usfirst.frc.team1318.robot.Autonomous.AutonomousControlData;
import org.usfirst.frc.team1318.robot.Autonomous.IAutonomousTask;

/**
 * ArmMacroTask:
 * 
 * This task runs the arm extend or arm retract macro, sleeping for a certain (provided) length of time.
 * 
 * @author Will
 *
 */
public class ArmMacroTask extends TimedAutonomousTask implements IAutonomousTask
{
    private final boolean extend;

    /**
     * Initializes a new ArmMacroTask
     * @param duration to perform the task in seconds
     * @param extend the arm (true) or retract the arm (false)
     */
    public ArmMacroTask(double duration, boolean extend)
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
        data.setArmMacroExtendState(this.extend);
        data.setArmMacroRetractState(!this.extend);
    }

    /**
     * Cancel the current task and clear control changes
     * @param data to which we should clear any updated control settings
     */
    @Override
    public void cancel(AutonomousControlData data)
    {
        super.cancel(data);

        data.setArmMacroExtendState(false);
        data.setArmMacroRetractState(false);
    }

    /**
     * End the current task and reset control changes appropriately
     * @param data to which we should apply updated settings
     */
    @Override
    public void end(AutonomousControlData data)
    {
        super.end(data);

        data.setArmMacroExtendState(false);
        data.setArmMacroRetractState(false);
    }
}
