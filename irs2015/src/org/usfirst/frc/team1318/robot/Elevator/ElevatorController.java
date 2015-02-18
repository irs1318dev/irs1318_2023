package org.usfirst.frc.team1318.robot.Elevator;

import org.usfirst.frc.team1318.robot.HardwareConstants;
import org.usfirst.frc.team1318.robot.TuningConstants;
import org.usfirst.frc.team1318.robot.Common.IController;
import org.usfirst.frc.team1318.robot.Common.IDriver;
import org.usfirst.frc.team1318.robot.Common.PIDHandler;
import org.usfirst.frc.team1318.robot.Common.SmartDashboardLogger;

import edu.wpi.first.wpilibj.Timer;

public class ElevatorController implements IController
{
    private static final String ENCODER_ZERO_OFFSET_LOG_KEY = "e.encoderZeroOffset";
    private static final String POSITION_GOAL_LOG_KEY = "e.positionGoal";

    private final ElevatorComponent component;
    private final IDriver driver;

    private enum ContainerMacroStates
    {
        STATE_0, STATE_1_LOWER, STATE_2_WAIT;
    }

    private ContainerMacroStates containerMacroState;

    private double baseLevel;
    private double position;
    private double encoderZeroOffset;

    private boolean usePID;
    private PIDHandler pidHandler;

    private double lastTime;
    private Timer timer;

    private boolean movingToBottom;
    private boolean ignoreSensors;

    private boolean elevatorSlowMode;

    public ElevatorController(IDriver driver, ElevatorComponent component)
    {
        this.component = component;
        this.driver = driver;

        this.usePID = true;
        this.createPIDHandler();

        this.ignoreSensors = false;

        this.baseLevel = HardwareConstants.ELEVATOR_FLOOR_HEIGHT;
        this.position = this.component.getEncoderDistance();
        this.encoderZeroOffset = 0;
        this.movingToBottom = false;  // move to bottom to calibrate the encoder offset on start

        this.timer = new Timer();
        this.timer.start();
        this.lastTime = this.timer.get();
        this.containerMacroState = ContainerMacroStates.STATE_0;

        this.elevatorSlowMode = false;
    }

    @Override
    public void update()
    {
        component.getThroughBeamBroken();

        boolean enforceNonPositive = false;
        boolean enforceNonNegative = false;
        double currentTime = this.timer.get();

        if (this.elevatorSlowMode && this.driver.getFastElevatorButton())
        {
            this.elevatorSlowMode = false;
            this.createPIDHandler();
        }
        else if (!this.elevatorSlowMode && this.driver.getSlowElevatorButton())
        {
            this.elevatorSlowMode = true;
            this.createPIDHandler();
        }

        // handle enabling/disabling PID (enabling PID takes precedence)
        if (this.driver.getElevatorPIDOn())
        {
            // create PID handler if we are changing modes...
            if (!this.usePID)
            {
                this.usePID = true;
                this.createPIDHandler();
                this.position = component.getEncoderDistance() + this.encoderZeroOffset;

            }
        }
        else if (this.driver.getElevatorPIDOff())
        {
            // remove PID handler if we are changing modes...
            if (this.usePID)
            {
                this.usePID = false;
                this.createPIDHandler();
            }
        }

        // check whether ignore or use sensors; using sensors takes precedence over ignoring them
        if (this.driver.getIgnoreElevatorSensors())
        {
            this.ignoreSensors = true;
        }
        else if (this.driver.getUseElevatorSensors())
        {
            this.ignoreSensors = false;
        }

        // set elevator base level here
        if (this.driver.getElevatorSetStateToFloorButton())
        {
            this.baseLevel = HardwareConstants.ELEVATOR_FLOOR_HEIGHT;
        }
        //        else if (this.driver.getElevatorSetStateToPlatformButton())
        //        {
        //            this.baseLevel = HardwareConstants.ELEVATOR_PLATFORM_HEIGHT;
        //        }
        else if (this.driver.getElevatorSetStateToStepButton())
        {
            this.baseLevel = HardwareConstants.ELEVATOR_STEP_HEIGHT;
        }

        if (this.driver.getZeroElevatorEncoder())
        {
            this.encoderZeroOffset = HardwareConstants.ELEVATOR_MIN_HEIGHT + this.component.getEncoderDistance();
            this.position -= this.encoderZeroOffset;
        }

        double powerLevel = 0.0;

        switch (this.containerMacroState)
        {
            case STATE_0:
                if (this.driver.getElevatorPickUpMacro())
                {
                    this.containerMacroState = ContainerMacroStates.STATE_1_LOWER;
                }
                break;
            case STATE_1_LOWER:
                if (!this.ignoreSensors)
                {
                    this.movingToBottom = true;
                    this.containerMacroState = ContainerMacroStates.STATE_2_WAIT;
                }
                else
                {
                    this.containerMacroState = ContainerMacroStates.STATE_0;
                }
                break;
            case STATE_2_WAIT:
                if (!this.movingToBottom)
                {
                    if (this.usePID)
                    {
                        this.position = HardwareConstants.ELEVATOR_1_TOTE_HEIGHT;
                    }
                    this.containerMacroState = ContainerMacroStates.STATE_0;
                }
                break;
        }

        if (this.containerMacroState == ContainerMacroStates.STATE_0)
        {
            SmartDashboardLogger.putNumber("e.macroState", 0);
        }
        else if (this.containerMacroState == ContainerMacroStates.STATE_1_LOWER)
        {
            SmartDashboardLogger.putNumber("e.macroState", 1);
        }
        else if (this.containerMacroState == ContainerMacroStates.STATE_2_WAIT)
        {
            SmartDashboardLogger.putNumber("e.macroState", 2);
        }

        // checks whether it is in a mode to move down until the sensor is triggered 
        if (this.driver.getElevatorMoveToBottom())
        {
            this.movingToBottom = true;
            this.containerMacroState = ContainerMacroStates.STATE_0;
        }

        if (this.movingToBottom)
        {
            if (this.usePID)
            {
                powerLevel = this.calculatePositionModePowerSetting(TuningConstants.ELEVATOR_BELOW_MINIMUM_POSITION);
            }
            else
            {
                powerLevel = -TuningConstants.ELEVATOR_OVERRIDE_POWER_LEVEL;
            }
        }

        // check for position normally
        // if usePID is true, calculate power-level using PID
        if (this.getActionButtonPressed() || !this.movingToBottom)
        {
            // if usePID is true, calculate velocity using PID.
            if (this.usePID)
            {
                // calculate position to set elevator
                this.position = this.getPositionShift();

                // reset in case position is less than minimum, or more then maximum
                this.position = Math.max(this.position, HardwareConstants.ELEVATOR_MIN_HEIGHT);
                this.position = Math.min(this.position, HardwareConstants.ELEVATOR_MAX_HEIGHT);

                powerLevel = this.calculatePositionModePowerSetting(this.position);
            }
            else
            {
                // if we are in non-PID mode, pressing neither the up nor down override buttons means we should stop applying power to the motor
                powerLevel = 0.0;
            }

            // we cancel moving-to-bottom mode if we press a button
            this.movingToBottom = false;
        }

        // if we hit the bottom or top, adjust the encoder zero offset
        // also note that we should enforce hardware safety requirements
        if (this.component.getBottomLimitSwitchValue() && !this.ignoreSensors)
        {
            this.encoderZeroOffset = HardwareConstants.ELEVATOR_MIN_HEIGHT + this.component.getEncoderDistance();

            if (this.movingToBottom)
            {
                this.position = 0.0;
                this.movingToBottom = false;
            }

            enforceNonNegative = true;
        }
        else if (this.component.getTopLimitSwitchValue() && !this.ignoreSensors)
        {
            this.containerMacroState = ContainerMacroStates.STATE_0;
            this.encoderZeroOffset = this.component.getEncoderDistance() - HardwareConstants.ELEVATOR_MAX_HEIGHT;
            this.position = HardwareConstants.ELEVATOR_MAX_HEIGHT;
            enforceNonPositive = true;
        }

        // if elevator up or down button is pushed, these take precedence over the normal controls 
        // Also, down override button takes precedence over the up override button.
        if (this.driver.getElevatorDownButton())
        {
            this.containerMacroState = ContainerMacroStates.STATE_0;
            // if usePID is true, calculate velocity using PID.
            if (this.usePID)
            {
                this.position -= TuningConstants.ELEVATOR_MAX_VELOCITY * (currentTime - this.lastTime);

                // reset in case position is less than minimum, or more then maximum
                this.position = Math.max(this.position, HardwareConstants.ELEVATOR_MIN_HEIGHT);
                this.position = Math.min(this.position, HardwareConstants.ELEVATOR_MAX_HEIGHT);

                powerLevel = this.calculatePositionModePowerSetting(this.position);
            }
            else
            {
                powerLevel = -TuningConstants.ELEVATOR_OVERRIDE_POWER_LEVEL;
            }

            this.movingToBottom = false;
        }
        else if (this.driver.getElevatorUpButton())
        {
            this.containerMacroState = ContainerMacroStates.STATE_0;
            // if usePID is true, calculate velocity using PID.
            if (this.usePID)
            {
                this.position += TuningConstants.ELEVATOR_MAX_VELOCITY * (currentTime - this.lastTime);

                // reset in case position is less than minimum, or more then maximum
                this.position = Math.max(this.position, HardwareConstants.ELEVATOR_MIN_HEIGHT);
                this.position = Math.min(this.position, HardwareConstants.ELEVATOR_MAX_HEIGHT);

                powerLevel = this.calculatePositionModePowerSetting(this.position);
            }
            else
            {
                powerLevel = TuningConstants.ELEVATOR_OVERRIDE_POWER_LEVEL;
            }

            this.movingToBottom = false;
        }

        //TODO: take out
        if (Math.abs(this.driver.getElevatorVelocityOverride()) > TuningConstants.ELEVATOR_DEAD_ZONE)
        {
            double velocityIntensity = this.adjustIntensity(this.driver.getElevatorVelocityOverride());

            this.position += TuningConstants.ELEVATOR_MAX_VELOCITY * velocityIntensity * (currentTime - this.lastTime);

            powerLevel = this.calculatePositionModePowerSetting(this.position);
        }

        // Safety requirement: don't go lower if we are hitting the bottom limit switch
        if (enforceNonPositive)
        {
            powerLevel = Math.min(powerLevel, 0);
        }

        // Safety requirement: don't go higher if we are hitting the upper limit switch
        if (enforceNonNegative)
        {
            powerLevel = Math.max(powerLevel, 0);
        }

        // Safety requirement: stop if the elevator stop button has been pressed
        if (this.driver.getStopElevatorButton())
        {
            this.containerMacroState = ContainerMacroStates.STATE_0;
            powerLevel = 0.0;

            // also disable PID when we stop the elevator - otherwise the next iteration will cause us 
            // to continue to try to hit the same position setpoint we were trying to hit earlier
            if (this.usePID)
            {
                this.usePID = false;
                this.createPIDHandler();
            }
        }

        SmartDashboardLogger.putNumber(ElevatorController.POSITION_GOAL_LOG_KEY, this.position);
        SmartDashboardLogger.putNumber(ElevatorController.ENCODER_ZERO_OFFSET_LOG_KEY, encoderZeroOffset);

        this.component.setMotorPowerLevel(powerLevel);

        this.lastTime = currentTime;

        if ((this.usePID && this.position < HardwareConstants.ELEVATOR_1_TOTE_HEIGHT))
        //|| (!this.usePID && !this.ignoreSensors && this.component.getBottomLimitSwitchValue()))
        {
            this.component.setLimitSwitchRelayValue(true);
        }
        else
        {
            this.component.setLimitSwitchRelayValue(false);
        }

        if (this.component.getThroughBeamBroken())
        {
            this.component.setThroughBeamRelayValue(true);
        }
        else
        {
            this.component.setThroughBeamRelayValue(false);
        }
    }

    /**
     * changes the desired state in response to new user input, otherwise returns current desired position 
     * @return desired position 
     */
    private double getPositionShift()
    {
        if (this.driver.getElevatorMoveTo0TotesButton())
        {
            this.movingToBottom = false;
            this.containerMacroState = ContainerMacroStates.STATE_0;
            return HardwareConstants.ELEVATOR_0_TOTE_HEIGHT + this.baseLevel;
        }
        else if (this.driver.getElevatorMoveTo1ToteButton())
        {
            this.movingToBottom = false;
            this.containerMacroState = ContainerMacroStates.STATE_0;
            return HardwareConstants.ELEVATOR_1_TOTE_HEIGHT + this.baseLevel;
        }
        else if (this.driver.getElevatorMoveTo2TotesButton())
        {
            this.movingToBottom = false;
            this.containerMacroState = ContainerMacroStates.STATE_0;
            return HardwareConstants.ELEVATOR_2_TOTE_HEIGHT + this.baseLevel;
        }
        else if (this.driver.getElevatorMoveTo3TotesButton())
        {
            this.movingToBottom = false;
            this.containerMacroState = ContainerMacroStates.STATE_0;
            return HardwareConstants.ELEVATOR_3_TOTE_HEIGHT + this.baseLevel;
        }
        else
        {
            // Note: we only adjust position if we change the totes level.
            // If we change only the base level, we just stay at the current position
            // until we change the totes level as well.
            return this.position;
        }
    }

    private boolean getActionButtonPressed()
    {
        return this.driver.getElevatorMoveTo0TotesButton() || this.driver.getElevatorMoveTo1ToteButton() ||
            this.driver.getElevatorMoveTo2TotesButton() || this.driver.getElevatorMoveTo3TotesButton();
    }

    @Override
    public void stop()
    {
        // stop the elevator's motor
        this.component.setMotorPowerLevel(0.0);
    }

    /**
     * Adjust the intensity of the input value
     * @param value to adjust
     * @return adjusted value
     */
    private double adjustIntensity(double value)
    {
        // we will use simple quadratic scaling to adjust input intensity
        if (value < 0)
        {
            return -value * value;
        }
        else
        {
            return value * value;
        }
    }

    private void createPIDHandler()
    {
        if (!this.usePID)
        {
            this.pidHandler = null;
        }
        else
        {
            if (this.elevatorSlowMode)
            {
                this.pidHandler = new PIDHandler(
                    "e.PID",
                    TuningConstants.ELEVATOR_POSITION_PID_KP_DEFAULT,
                    TuningConstants.ELEVATOR_POSITION_PID_KI_DEFAULT,
                    TuningConstants.ELEVATOR_POSITION_PID_KD_DEFAULT,
                    TuningConstants.ELEVATOR_POSITION_PID_KF_DEFAULT,
                    -TuningConstants.ELEVATOR_MAX_POWER_LEVEL / 2,
                    TuningConstants.ELEVATOR_MAX_POWER_LEVEL / 2);
            }
            else
            {
                this.pidHandler = new PIDHandler(
                    "e.PID",
                    TuningConstants.ELEVATOR_POSITION_PID_KP_DEFAULT,
                    TuningConstants.ELEVATOR_POSITION_PID_KI_DEFAULT,
                    TuningConstants.ELEVATOR_POSITION_PID_KD_DEFAULT,
                    TuningConstants.ELEVATOR_POSITION_PID_KF_DEFAULT,
                    -TuningConstants.ELEVATOR_MAX_POWER_LEVEL,
                    TuningConstants.ELEVATOR_MAX_POWER_LEVEL);
            }
        }
    }

    private double calculatePositionModePowerSetting(double desired)
    {
        double current = this.component.getEncoderDistance() - this.encoderZeroOffset;
        return this.pidHandler.calculatePosition(desired, current);
    }
}
