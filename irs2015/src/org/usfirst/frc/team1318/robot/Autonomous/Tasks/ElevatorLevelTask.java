package org.usfirst.frc.team1318.robot.Autonomous.Tasks;

import org.usfirst.frc.team1318.robot.HardwareConstants;
import org.usfirst.frc.team1318.robot.Autonomous.AutonomousConstants;
import org.usfirst.frc.team1318.robot.Autonomous.AutonomousControlData;
import org.usfirst.frc.team1318.robot.Autonomous.IAutonomousTask;
import org.usfirst.frc.team1318.robot.Elevator.ElevatorComponent;

/**
 * ElevatorFloorTask:
 * 
 * This task tells the elevator to go to a certain position until we detect that we have hit that position.
 * 
 * @author Will
 *
 */
public class ElevatorLevelTask implements IAutonomousTask
{
    private final ElevatorComponent elevatorComponent;
    private final int toteLevel;
    private final int baseLevel;
    private final boolean fast;

    private boolean hasReachedLevel;

    /**
     * Initializes a new ElevatorFloorTask
     * @param elevatorComponent to use to detect whether we have hit our bottom limit switch
     * @param toteLevel - whether to go to 0, 1, 2, or 3 tote height.
     * @param baseLevel - whether to go to floor (0), scoring platform (1), or step (2)
     * @param fast indicates that we should switch into fast mode
     */
    public ElevatorLevelTask(ElevatorComponent elevatorComponent, int toteLevel, int baseLevel, boolean fast)
    {
        this.elevatorComponent = elevatorComponent;

        this.toteLevel = toteLevel;
        this.baseLevel = baseLevel;
        this.fast = fast;
    }

    /**
     * Begin the current task
     */
    @Override
    public void begin()
    {
        this.hasReachedLevel = false;
    }

    /**
     * Run an iteration of the current task and apply any control changes 
     * @param data to which we should apply updated settings
     */
    @Override
    public void update(AutonomousControlData data)
    {
        double expectedPosition = this.elevatorComponent.getEncoderZeroOffset();

        boolean tote0 = false;
        boolean tote1 = false;
        boolean tote2 = false;
        boolean tote3 = false;
        switch (this.toteLevel)
        {
            case 0:
                expectedPosition += HardwareConstants.ELEVATOR_0_TOTE_HEIGHT;
                tote0 = true;
                break;

            case 1:
                expectedPosition += HardwareConstants.ELEVATOR_1_TOTE_HEIGHT;
                tote1 = true;
                break;

            case 2:
                expectedPosition += HardwareConstants.ELEVATOR_2_TOTE_HEIGHT;
                tote2 = true;
                break;

            case 3:
                expectedPosition += HardwareConstants.ELEVATOR_3_TOTE_HEIGHT;
                tote3 = true;
                break;
        }

        boolean floor = false;
        boolean platform = false;
        boolean step = false;
        switch (this.baseLevel)
        {
            case 0:
                expectedPosition += HardwareConstants.ELEVATOR_FLOOR_HEIGHT;
                floor = true;
                break;

            case 1:
                platform = true;
                break;

            case 2:
                expectedPosition += HardwareConstants.ELEVATOR_STEP_HEIGHT;
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

        double delta = this.elevatorComponent.getEncoderDistance() - expectedPosition;
        if (delta < AutonomousConstants.ELEVATOR_POSITIONAL_ACCEPTABLE_DELTA
            && delta > -AutonomousConstants.ELEVATOR_POSITIONAL_ACCEPTABLE_DELTA)
        {
            this.hasReachedLevel = true;
        }
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

    /**
     * Checks whether this task has completed, or whether it should continue being processed
     * @return true if we should continue onto the next task, otherwise false (to keep processing this task)
     */
    @Override
    public boolean hasCompleted()
    {
        return this.hasReachedLevel;
    }
}
