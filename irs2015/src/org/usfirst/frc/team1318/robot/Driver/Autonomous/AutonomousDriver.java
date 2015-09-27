package org.usfirst.frc.team1318.robot.Driver.Autonomous;

import java.util.HashMap;
import java.util.Map;

import org.usfirst.frc.team1318.robot.Driver.Driver;
import org.usfirst.frc.team1318.robot.Driver.IControlTask;
import org.usfirst.frc.team1318.robot.Driver.Operation;
import org.usfirst.frc.team1318.robot.Driver.States.OperationState;

/**
 * Driver for autonomous mode.  Autonomous driver acts as the operator of the robot,
 * telling it what actions to perform as determined by the current task and tasks that have
 * come before it that intentionally don't reset their state.
 * 
 * @author Will
 *
 */
public class AutonomousDriver extends Driver
{
    // logging constants
    private final IControlTask autonomousTask;

    private boolean hasBegun;
    private boolean hasEnded;

    private final Map<Operation, OperationState> operationStateMap;

    /**
     * Initializes a new AutonomousDriver
     * @param autonomousTask to execute as a part of this driver
     */
    public AutonomousDriver(IControlTask autonomousTask)
    {
        this.autonomousTask = autonomousTask;
        this.operationStateMap = new HashMap<Operation, OperationState>(this.operationSchema.size());

        this.hasBegun = false;
        this.hasEnded = false;
    }

    /**
     * Tell the driver that some time has passed
     */
    @Override
    public void update()
    {
        if (!this.hasEnded)
        {
            if (!this.hasBegun)
            {
                // if we haven't begun, begin
                this.autonomousTask.begin();
                this.hasBegun = true;
            }

            if (this.autonomousTask.hasCompleted())
            {
                // if we shouldn't continue, end the task
                this.autonomousTask.end();
                this.hasEnded = true;
            }
            else
            {
                // run the current task and apply the result to the state
                this.autonomousTask.update();
            }
        }
    }

    /**
     * Tell the driver that operation is stopping
     */
    @Override
    public void stop()
    {
        if (this.autonomousTask != null)
        {
            this.autonomousTask.stop();
            this.hasEnded = true;
        }
    }

    /**
     * Get a boolean indicating whether the current digital operation is enabled
     * @param digitalOperation to get
     * @return the current value of the digital operation
     */
    @Override
    public boolean getDigital(Operation digitalOperation)
    {
        return false;
    }

    /**
     * Get a double between -1.0 and 1.0 indicating the current value of the analog operation
     * @param analogOperation to get
     * @return the current value of the analog operation
     */
    @Override
    public double getAnalog(Operation analogOperation)
    {
        return 0.0;
    }
}
