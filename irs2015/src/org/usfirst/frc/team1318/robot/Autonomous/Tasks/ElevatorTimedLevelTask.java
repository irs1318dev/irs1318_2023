package org.usfirst.frc.team1318.robot.Autonomous.Tasks;

import org.usfirst.frc.team1318.robot.Autonomous.AutonomousControlData;
import org.usfirst.frc.team1318.robot.Autonomous.IAutonomousTask;

/**
 * ElevatorTimedLevelTask:
 * 
 * This task tells the elevator to go to a certain position until we detect that we have hit that position.
 * 
 * @author Will
 *
 */
public class ElevatorTimedLevelTask extends TimedAutonomousTask implements IAutonomousTask
{
    private final int toteLevel;
    private final int baseLevel;
    private final boolean fast;

    /**
     * Initializes a new ElevatorFloorTask
     * @param duration to perform the task in seconds
     * @param toteLevel - whether to go to 0, 1, 2, or 3 tote height.
     * @param baseLevel - whether to go to floor (0), scoring platform (1), or step (2)
     * @param fast indicates that we should switch into fast mode
     */
    public ElevatorTimedLevelTask(double duration, int toteLevel, int baseLevel, boolean fast)
    {
        super(duration);

        this.toteLevel = toteLevel;
        this.baseLevel = baseLevel;
        this.fast = fast;
    }

    /**
     * Run an iteration of the current task and apply any control changes 
     * @param data to which we should apply updated settings
     */
    @Override
    public void update(AutonomousControlData data)
    {
        boolean tote0 = false;
        boolean tote1 = false;
        boolean tote2 = false;
        boolean tote3 = false;
        switch (this.toteLevel)
        {
            case 0:
                tote0 = true;
                break;

            case 1:
                tote1 = true;
                break;

            case 2:
                tote2 = true;
                break;

            case 3:
                tote3 = true;
                break;
        }

        boolean floor = false;
        boolean platform = false;
        boolean step = false;
        switch (this.baseLevel)
        {
            case 0:
                floor = true;
                break;

            case 1:
                platform = true;
                break;

            case 2:
                step = true;
                break;
        }

        data.setElevatorSetStateToFloor(floor);
        data.setElevatorSetStateToPlatform(platform);
        data.setElevatorSetStateToStep(step);

        data.setElevatorMoveTo0Totes(tote0);
        data.setElevatorMoveTo1Tote(tote1);
        data.setElevatorMoveTo2Totes(tote2);
        data.setElevatorMoveTo3Totes(tote3);

        data.setElevatorFastButton(this.fast);
    }

    /**
    * Cancel the current task and clear control changes
    * @param data to which we should clear any updated control settings
    */
    @Override
    public void cancel(AutonomousControlData data)
    {
        data.setElevatorSetStateToFloor(false);
        data.setElevatorSetStateToPlatform(false);
        data.setElevatorSetStateToStep(false);

        data.setElevatorMoveTo0Totes(false);
        data.setElevatorMoveTo1Tote(false);
        data.setElevatorMoveTo2Totes(false);
        data.setElevatorMoveTo3Totes(false);

        data.setElevatorFastButton(false);
    }

    /**
     * End the current task and reset control changes appropriately
     * @param data to which we should apply updated settings
     */
    @Override
    public void end(AutonomousControlData data)
    {
        data.setElevatorSetStateToFloor(false);
        data.setElevatorSetStateToPlatform(false);
        data.setElevatorSetStateToStep(false);

        data.setElevatorMoveTo0Totes(false);
        data.setElevatorMoveTo1Tote(false);
        data.setElevatorMoveTo2Totes(false);
        data.setElevatorMoveTo3Totes(false);

        data.setElevatorFastButton(false);
    }
}
