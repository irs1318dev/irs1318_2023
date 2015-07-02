package org.usfirst.frc.team1318.robot.Autonomous.Tasks;

import org.usfirst.frc.team1318.robot.Autonomous.AutonomousControlData;
import org.usfirst.frc.team1318.robot.Autonomous.IAutonomousTask;
import org.usfirst.frc.team1318.robot.Elevator.ElevatorComponent;

/**
 * CollectToteTask:
 * 
 * This task:
 * 1. Runs the intake until it notices that the through beam sensor has been broken, then
 * 2. Stops the intake and tells the elevator to go to the bottom (move to bottom) until the bottom limit switch is hit, then 
 * 3. Tells the elevator to go up to the 3rd level
 * 
 * @author Will
 *
 */
public class CollectToteTask implements IAutonomousTask
{
    private final ElevatorComponent elevatorComponent;

    private boolean hasDetectedThroughBeamBroken;

    /**
     * Initializes a new CollectToteTask
     * @param elevatorComponent to use to detect whether we have broken the through beam
     */
    public CollectToteTask(ElevatorComponent elevatorComponent)
    {
        this.elevatorComponent = elevatorComponent;
        this.hasDetectedThroughBeamBroken = false;
    }

    /**
     * Begin the current task
     */
    @Override
    public void begin()
    {
        this.hasDetectedThroughBeamBroken = false;
    }

    /**
     * Run an iteration of the current task and apply any control changes 
     * @param data to which we should apply updated settings
     */
    @Override
    public void update(AutonomousControlData data)
    {
        if (!this.hasDetectedThroughBeamBroken)
        {
            if (this.elevatorComponent.getThroughBeamBroken())
            {
                this.hasDetectedThroughBeamBroken = true;
            }
        }

        data.setIntakeForwardState(!this.hasDetectedThroughBeamBroken);
    }

    /**
     * Cancel the current task and clear control changes
     * @param data to which we should clear any updated control settings
     */
    @Override
    public void cancel(AutonomousControlData data)
    {
        data.setIntakeForwardState(false);
    }

    /**
     * End the current task and reset control changes appropriately
     * @param data to which we should apply updated settings
     */
    @Override
    public void end(AutonomousControlData data)
    {
        data.setIntakeForwardState(false);
    }

    /**
     * Checks whether this task has completed, or whether it should continue being processed
     * @return true if we should continue onto the next task, otherwise false (to keep processing this task)
     */
    @Override
    public boolean hasCompleted()
    {
        return this.hasDetectedThroughBeamBroken;
    }
}
