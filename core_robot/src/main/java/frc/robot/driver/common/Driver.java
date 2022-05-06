package frc.robot.driver.common;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

import javax.inject.Singleton;

import frc.robot.LoggingKey;
import frc.robot.TuningConstants;
import frc.robot.common.LoggingManager;
import frc.robot.common.SetHelper;
import frc.robot.common.robotprovider.*;
import frc.robot.driver.*;
import frc.robot.driver.common.descriptions.*;
import frc.robot.driver.common.states.*;

import com.google.inject.Inject;
import com.google.inject.Injector;

/**
 * Driver that represents something that operates the robot.  This is either autonomous or teleop/user driver.
 *
 */
@Singleton
public class Driver implements IDriver
{
    private final ILogger logger;

    protected final Injector injector;
    protected final Map<IOperation, OperationState> operationStateMap;

    private final IJoystick[] joysticks;

    private final Map<Shift, ShiftDescription> shiftMap;
    private final Map<MacroOperation, IMacroOperationState> macroStateMap;

    private final AutonomousRoutineSelector routineSelector;
    private IControlTask autonomousTask;

    private RobotMode currentMode;

    /**
     * Initializes a new Driver
     * @param logger used to log data to the dashboard
     * @param injector used to retrieve the components to utilize within the robot
     * @param buttonMap to control the mapping of joysticks to the corresponding operations
     * @param provider to retrieve abstracted robot joysticks
     */
    @Inject
    public Driver(
        LoggingManager logger,
        Injector injector,
        IButtonMap buttonMap,
        IRobotProvider provider)
    {
        this.logger = logger;
        this.injector = injector;

        HashSet<UserInputDevice> devices = new HashSet<UserInputDevice>();
        AnalogOperationDescription[] analogOperationSchema = buttonMap.getAnalogOperationSchema();
        DigitalOperationDescription[] digitalOperationSchema = buttonMap.getDigitalOperationSchema();

        DigitalOperation[] digitalOperations = DigitalOperation.values();
        AnalogOperation[] analogOperations = AnalogOperation.values();

        this.operationStateMap = new HashMap<IOperation, OperationState>(analogOperations.length + digitalOperations.length);
        for (DigitalOperationDescription description : digitalOperationSchema)
        {
            devices.add(description.getUserInputDevice());
            this.operationStateMap.put(description.getOperation(), new DigitalOperationState(description));
        }

        for (DigitalOperation operation : digitalOperations)
        {
            if (!this.operationStateMap.containsKey(operation))
            {
                this.operationStateMap.put(
                    operation,
                    new DigitalOperationState(new DigitalOperationDescription(operation)));
            }
        }

        for (AnalogOperationDescription description : analogOperationSchema)
        {
            devices.add(description.getUserInputDevice());
            this.operationStateMap.put(description.getOperation(), new AnalogOperationState(description));
        }

        for (AnalogOperation operation : analogOperations)
        {
            if (!this.operationStateMap.containsKey(operation))
            {
                this.operationStateMap.put(
                    operation,
                    new AnalogOperationState(new AnalogOperationDescription(operation)));
            }
        }

        this.routineSelector = injector.getInstance(AutonomousRoutineSelector.class);

        ShiftDescription[] shiftSchema = buttonMap.getShiftSchema();
        this.shiftMap = new HashMap<Shift, ShiftDescription>();
        for (ShiftDescription description : shiftSchema)
        {
            this.shiftMap.put(description.getShift(), description);
        }

        this.macroStateMap = new HashMap<MacroOperation, IMacroOperationState>();
        MacroOperationDescription[] macroSchema = buttonMap.getMacroOperationSchema();
        for (MacroOperationDescription description : macroSchema)
        {
            devices.add(description.getUserInputDevice());
            this.macroStateMap.put(
                (MacroOperation)description.getOperation(),
                new MacroOperationState(
                    description,
                    this.operationStateMap,
                    this.injector));
        }

        ButtonMapVerifier.Verify(buttonMap);

        this.joysticks = new IJoystick[UserInputDevice.MaxCount.getId()];
        for (UserInputDevice device : UserInputDevice.values())
        {
            if (device != UserInputDevice.None &&
                devices.contains(device))
            {
                int id = device.getId();
                this.joysticks[id] = provider.getJoystick(id);
            }
        }

        this.currentMode = RobotMode.Disabled;

        // initialize the path manager and load all of the paths
        injector.getInstance(PathManager.class);
    }

    /**
     * Checks whether the driver is in autonomous mode
     */
    @Override
    public RobotMode getMode()
    {
        return this.currentMode;
    }

    /**
     * Tell the driver that some time has passed
     */
    @Override
    public void update()
    {
        this.logger.logString(LoggingKey.DriverMode, this.currentMode.toString());

        // keep track of macros that were running before we checked user input...
        Set<MacroOperation> previouslyActiveMacroOperations = new HashSet<MacroOperation>();
        for (MacroOperation macroOperation : this.macroStateMap.keySet())
        {
            IMacroOperationState macroState = this.macroStateMap.get(macroOperation);
            if (macroState.getIsActive())
            {
                previouslyActiveMacroOperations.add(macroOperation);
            }
        }

        // check inputs and update shifts based on it...
        int shiftIndex = 0;
        Shift[] activeShiftList = new Shift[this.shiftMap.size()];
        for (Shift shift : this.shiftMap.keySet())
        {
            ShiftDescription shiftDescription = this.shiftMap.get(shift);
            if (this.currentMode != RobotMode.Autonomous && shiftDescription.checkInput(this.joysticks))
            {
                activeShiftList[shiftIndex++] = shift;
            }
        }

        Shift activeShifts = Shift.Union(activeShiftList);

        // check user inputs for various operations (non-macro) and keep track of:
        // operations that were interrupted already, and operations that were modified by user input in this update
        Set<IOperation> modifiedOperations = new HashSet<IOperation>();
        Set<IOperation> interruptedOperations = new HashSet<IOperation>();
        for (IOperation operation : this.operationStateMap.keySet())
        {
            OperationState opState = this.operationStateMap.get(operation);
            boolean receivedInput = this.currentMode != RobotMode.Autonomous && opState.checkInput(this.joysticks, activeShifts);
            if (receivedInput)
            {
                modifiedOperations.add(operation);
            }

            if (this.operationStateMap.get(operation).getIsInterrupted())
            {
                interruptedOperations.add(operation);
            }
        }

        // check user inputs for various macro operations
        // also keep track of modified and active macro operations, and how macro operations and operations link together
        Set<MacroOperation> activeMacroOperations = new HashSet<MacroOperation>();
        Map<IOperation, Set<MacroOperation>> activeMacroOperationMap = new HashMap<IOperation, Set<MacroOperation>>();
        for (MacroOperation macroOperation : this.macroStateMap.keySet())
        {
            IMacroOperationState macroState = this.macroStateMap.get(macroOperation);
            if (this.currentMode != RobotMode.Autonomous)
            {
                macroState.checkInput(this.joysticks, activeShifts);
            }

            if (macroState.getIsActive())
            {
                activeMacroOperations.add(macroOperation);

                for (IOperation affectedOperation : macroState.getMacroCancelOperations())
                {
                    Set<MacroOperation> relevantMacroOperations = activeMacroOperationMap.get(affectedOperation);
                    if (relevantMacroOperations == null)
                    {
                        relevantMacroOperations = new HashSet<MacroOperation>();
                        activeMacroOperationMap.put(affectedOperation, relevantMacroOperations);
                    }

                    relevantMacroOperations.add(macroOperation);
                }
            }
        }

        // Determine the list of macro operations to cancel.  Only keep macros that:
        // 1. have not been usurped by a user action
        // 2. have not been usurped by a new macro (i.e. that was started in this round)
        // 3. are new macros that do not overlap with other new macros
        Set<MacroOperation> macroOperationsToCancel = new HashSet<MacroOperation>();
        for (IOperation operation : activeMacroOperationMap.keySet())
        {
            Set<MacroOperation> relevantMacroOperations = activeMacroOperationMap.get(operation);
            if (modifiedOperations.contains(operation))
            {
                // disobeys rule #1:
                // (macro usurped by user action)
                macroOperationsToCancel.addAll(relevantMacroOperations);
            }
            else if (relevantMacroOperations.size() > 1)
            {
                Set<MacroOperation> newRelevantMacroOperations = SetHelper.<MacroOperation>RelativeComplement(previouslyActiveMacroOperations, relevantMacroOperations);
                if (newRelevantMacroOperations.size() > 1)
                {
                    // disobeys rule #3:
                    // (there are 2 or more active macros that weren't previously active)
                    macroOperationsToCancel.addAll(relevantMacroOperations);
                }
                else
                {
                    // some disobey rule #2 (remove only those that were previously active, and not the 1 that is newly active...)
                    macroOperationsToCancel.addAll(SetHelper.<MacroOperation>RelativeComplement(newRelevantMacroOperations, relevantMacroOperations));
                }
            }
        }

        // cancel macros that didn't follow the rules list above
        for (MacroOperation macroOperationToCancel : macroOperationsToCancel)
        {
            this.macroStateMap.get(macroOperationToCancel).setIsInterrupted(true);
            activeMacroOperations.remove(macroOperationToCancel);
        }

        // first, run all of the inactive macros (to clear any old interrupts)...
        Set<MacroOperation> inactiveMacroOperations = SetHelper.<MacroOperation>RelativeComplement(activeMacroOperations, this.macroStateMap.keySet());
        for (MacroOperation macroOperation : inactiveMacroOperations)
        {
            this.macroStateMap.get(macroOperation).run();
        }

        // second, run all of the active macros (which could add interrupts that were cleared in the previous phase)...
        // while we're doing that, grab the names of the macros for logging
        int i = 0;
        String[] macroStrings = new String[activeMacroOperations.size()];
        for (MacroOperation macroOperation : activeMacroOperations)
        {
            macroStrings[i++] = macroOperation.toString();
            this.macroStateMap.get(macroOperation).run();
        }

        this.logger.logString(LoggingKey.DriverActiveMacros, String.join(", ", macroStrings));
        this.logger.logString(LoggingKey.DriverActiveShifts, activeShifts.toString());
    }

    /**
     * Tell the driver that operation is stopping
     */
    @Override
    public void stop()
    {
        this.currentMode = RobotMode.Disabled;

        if (this.macroStateMap.containsKey(MacroOperation.AutonomousRoutine))
        {
            this.macroStateMap.remove(MacroOperation.AutonomousRoutine);
        }

        // cancel all interruption of buttons:
        for (OperationState state : this.operationStateMap.values())
        {
            state.setIsInterrupted(false);
        }

        // cancel all ongoing macros:
        for (IMacroOperationState macroOperationState : this.macroStateMap.values())
        {
            macroOperationState.cancel();
        }
    }

    /**
     * Starts the autonomous period of the match (e.g. begins auto routine)
     */
    @Override
    public void startMode(RobotMode mode)
    {
        this.currentMode = mode;

        this.autonomousTask = this.routineSelector.selectRoutine(mode);
        if (this.autonomousTask != null)
        {
            this.autonomousTask.initialize(this.operationStateMap, injector);
            this.macroStateMap.put(
                MacroOperation.AutonomousRoutine,
                new AutonomousOperationState(this.autonomousTask, this.operationStateMap));
        }
    }

    /**
     * Get a boolean indicating whether the current digital operation is enabled
     * @param digitalOperation to get
     * @return the current value of the digital operation
     */
    public boolean getDigital(DigitalOperation digitalOperation)
    {
        OperationState state = this.operationStateMap.get(digitalOperation);
        if (!(state instanceof DigitalOperationState))
        {
            if (TuningConstants.THROW_EXCEPTIONS)
            {
                throw new RuntimeException("not a digital operation!");
            }

            return false;
        }

        DigitalOperationState digitalState = (DigitalOperationState)state;
        return digitalState.getState();
    }

    /**
     * Get a double between -1.0 and 1.0 indicating the current value of the analog operation
     * @param analogOperation to get
     * @return the current value of the analog operation
     */
    public double getAnalog(AnalogOperation analogOperation)
    {
        OperationState state = this.operationStateMap.get(analogOperation);
        if (!(state instanceof AnalogOperationState))
        {
            if (TuningConstants.THROW_EXCEPTIONS)
            {
                throw new RuntimeException("not an analog operation!");
            }

            return 0.0;
        }

        AnalogOperationState analogState = (AnalogOperationState)state;
        return analogState.getState();
    }
}
