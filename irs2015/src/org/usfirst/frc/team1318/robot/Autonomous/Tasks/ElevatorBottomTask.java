package org.usfirst.frc.team1318.robot.Autonomous.Tasks;

import org.usfirst.frc.team1318.robot.Autonomous.AutonomousControlData;
import org.usfirst.frc.team1318.robot.Autonomous.IAutonomousTask;
import org.usfirst.frc.team1318.robot.Elevator.ElevatorComponent;

/**
 * ElevatorBottomTask:
 * 
 * This task tells the elevator to go to the bottom (move to bottom) until the bottom limit switch is hit.
 * 
 * @author Will
 *
 */
public class ElevatorBottomTask implements IAutonomousTask
{
    private final ElevatorComponent elevatorComponent;

    private boolean hasHitBottomLimitSwitch;

    /**
     * Initializes a new ElevatorBottomTask
     * @param elevatorComponent to use to detect whether we have hit our bottom limit switch
     */
    public ElevatorBottomTask(ElevatorComponent elevatorComponent)
    {
        this.elevatorComponent = elevatorComponent;

        this.hasHitBottomLimitSwitch = false;
    }

    /**
     * Begin the current task
     */
    @Override
    public void begin()
    {
        this.hasHitBottomLimitSwitch = false;
    }

    /**
     * Run an iteration of the current task and apply any control changes 
     * @param data to which we should apply updated settings
     */
    @Override
    public void update(AutonomousControlData data)
    {
        if (this.elevatorComponent.getBottomLimitSwitchValue())
        {
            this.hasHitBottomLimitSwitch = true;
        }

        data.setElevatorMoveToBottomState(!this.hasHitBottomLimitSwitch);
    }

    /**
    * Cancel the current task and clear control changes
    * @param data to which we should clear any updated control settings
    */
    @Override
    public void cancel(AutonomousControlData data)
    {
        data.setElevatorMoveToBottomState(false);
    }

    /**
     * End the current task and reset control changes appropriately
     * @param data to which we should apply updated settings
     */
    @Override
    public void end(AutonomousControlData data)
    {
        data.setElevatorMoveToBottomState(false);
    }

    /**
     * Checks whether this task has completed, or whether it should continue being processed
     * @return true if we should continue onto the next task, otherwise false (to keep processing this task)
     */
    @Override
    public boolean hasCompleted()
    {
        return this.hasHitBottomLimitSwitch;
    }
}
