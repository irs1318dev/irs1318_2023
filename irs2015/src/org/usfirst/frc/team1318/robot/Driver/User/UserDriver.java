package org.usfirst.frc.team1318.robot.Driver.User;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.usfirst.frc.team1318.robot.Driver.Driver;
import org.usfirst.frc.team1318.robot.Driver.JoystickButtonConstants;
import org.usfirst.frc.team1318.robot.Driver.Operation;
import org.usfirst.frc.team1318.robot.Driver.States.AnalogOperationState;
import org.usfirst.frc.team1318.robot.Driver.States.DigitalOperationState;
import org.usfirst.frc.team1318.robot.Driver.States.MacroOperationState;
import org.usfirst.frc.team1318.robot.Driver.States.OperationState;

import edu.wpi.first.wpilibj.Joystick;

/**
 * Driver for teleop mode.  User driver translates current toggle state and joystick state into
 * the specific actions that should be taken by the robot.
 * 
 * @author Will
 *
 */
public class UserDriver extends Driver
{
    private final Joystick joystickDriver;
    private final Joystick joystickCoDriver;

    private final Map<Operation, OperationState> operationStateMap;
    private final List<MacroOperationState> macroStates;

    /**
     * Initializes a new UserDriver
     */
    public UserDriver()
    {
        this.joystickDriver = new Joystick(JoystickButtonConstants.JOYSTICK_DRIVER_PORT);
        this.joystickCoDriver = new Joystick(JoystickButtonConstants.JOYSTICK_CO_DRIVER_PORT);

        this.operationStateMap = new HashMap<Operation, OperationState>();
        for (Operation operation : this.operationSchema.keySet())
        {
            this.operationStateMap.put(operation, OperationState.createFromDescription(this.operationSchema.get(operation)));
        }

        this.macroStates = new ArrayList<MacroOperationState>();
    }

    /**
     * Tell the driver that some time has passed
     */
    @Override
    public void update()
    {
        List<Operation> changedOperations = new ArrayList<Operation>();
        for (Operation operation : this.operationStateMap.keySet())
        {
            boolean receivedInput = this.operationStateMap.get(operation).checkUserInput(this.joystickDriver, this.joystickCoDriver);
            if (receivedInput)
            {
                changedOperations.add(operation);
            }
        }

        for (MacroOperationState macroState : this.macroStates)
        {
            macroState.checkUserInput(this.joystickDriver, this.joystickCoDriver);
        }
    }

    /**
     * Tell the driver that operation is stopping
     */
    @Override
    public void stop()
    {
    }

    /**
     * Get a boolean indicating whether the current digital operation is enabled
     * @param digitalOperation to get
     * @return the current value of the digital operation
     */
    @Override
    public boolean getDigital(Operation digitalOperation)
    {
        OperationState state = this.operationStateMap.get(digitalOperation);
        if (!(state instanceof DigitalOperationState))
        {
            throw new RuntimeException("not a digital operation!");
        }

        DigitalOperationState digitalState = (DigitalOperationState)state;
        return digitalState.getState();
    }

    /**
     * Get a double between -1.0 and 1.0 indicating the current value of the analog operation
     * @param analogOperation to get
     * @return the current value of the analog operation
     */
    @Override
    public double getAnalog(Operation analogOperation)
    {
        OperationState state = this.operationStateMap.get(analogOperation);
        if (!(state instanceof AnalogOperationState))
        {
            throw new RuntimeException("not an analog operation!");
        }

        AnalogOperationState analogState = (AnalogOperationState)state;
        return analogState.getState();
    }
}
