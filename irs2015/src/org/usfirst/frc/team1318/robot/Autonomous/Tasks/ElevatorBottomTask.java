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
    private ElevatorComponent elevatorComponent;

    private boolean hasHitBottomLimitSwitch;

    public ElevatorBottomTask(ElevatorComponent elevatorComponent)
    {
        this.elevatorComponent = elevatorComponent;

        this.hasHitBottomLimitSwitch = false;
    }

    @Override
    public void begin()
    {
        this.hasHitBottomLimitSwitch = false;
    }

    @Override
    public void update(AutonomousControlData data)
    {
        if (this.elevatorComponent.getBottomLimitSwitchValue())
        {
            this.hasHitBottomLimitSwitch = true;
        }

        data.setElevatorMoveToBottomState(!this.hasHitBottomLimitSwitch);
    }

    @Override
    public void cancel(AutonomousControlData data)
    {
        data.setElevatorMoveToBottomState(false);
    }

    @Override
    public void end(AutonomousControlData data)
    {
        data.setElevatorMoveToBottomState(false);
    }

    @Override
    public boolean shouldContinueProcessingTask()
    {
        return this.hasHitBottomLimitSwitch;
    }
}
