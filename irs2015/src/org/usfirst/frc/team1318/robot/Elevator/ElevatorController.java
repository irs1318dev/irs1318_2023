package org.usfirst.frc.team1318.robot.Elevator;

import org.usfirst.frc.team1318.robot.Common.IController;
import org.usfirst.frc.team1318.robot.Common.IDriver;
import org.usfirst.frc.team1318.robot.Common.PIDHandler;

import edu.wpi.first.wpilibj.Preferences;

public class ElevatorController implements IController
{
    private PIDHandler pidHandler;
    private boolean useVelocityPID;
    private boolean usePID;
    private double state;
    private double position;

    private final ElevatorComponent component;
    private final IDriver driver;

    public static final double POWERLEVEL_MIN = -.5;
    public static final double POWERLEVEL_MAX = .5;

    private static final double FLOOR = 0;
    private static final double PLATFORM = -1;
    private static final double STEP = -1;

    private static final double TOTE_0 = -1;
    private static final double TOTE_1 = -1;
    private static final double TOTE_2 = -1;
    private static final double TOTE_3 = -1;

    private static final double OVERRIDE = 0.5;
    private static final double MINIMUM_HEIGHT = 0.0;
    private static final double MAXIMUM_HEIGHT = 0.0;

    public ElevatorController(ElevatorComponent component, IDriver driver)
    {
        this.component = component;
        this.driver = driver;

        this.useVelocityPID = false;
        this.usePID = true;

        this.state = ElevatorController.FLOOR;
        this.position = 0;
    }

    @Override
    public void update()
    {
        // set elevator state here
        if (this.driver.getElevatorSetStateToFloorButton())
        {
            this.state = ElevatorController.FLOOR;
        }
        else if (this.driver.getElevatorSetStateToPlatformButton())
        {
            this.state = ElevatorController.PLATFORM;
        }
        else if (this.driver.getElevatorSetStateToStepButton())
        {
            this.state = ElevatorController.STEP;
        }

        // handle enabling/disabling PID (enabling PID takes precedence)
        if (this.driver.getElevatorPIDOn())
        {
            // create PID handler if we are changing modes...
            if (!this.usePID)
            {
                this.usePID = true;
                this.createPIDHandler();
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

        // if elevator up or down button is pushed, do not deal with positional elevator buttons
        // down override button takes precedence over the up override button.
        if (this.driver.getElevatorDownButton())
        {
            // if usePID is true, calculate velocity using PID.
            if (this.usePID)
            {
                // recreate PID handler if we are changing modes...
                if (!this.useVelocityPID)
                {
                    this.useVelocityPID = true;
                    this.createPIDHandler();
                }

                this.component.setMotorPowerLevel(this.calculateVelocityModePowerSetting(-ElevatorController.OVERRIDE));
            }
            else
            {
                this.component.setMotorPowerLevel(-ElevatorController.OVERRIDE);
            }
        }
        else if (this.driver.getElevatorUpButton())
        {
            // if usePID is true, calculate velocity using PID.
            if (this.usePID)
            {
                // recreate PID handler if we are changing modes...
                if (!this.useVelocityPID)
                {
                    this.useVelocityPID = true;
                    this.createPIDHandler();
                }

                this.component.setMotorPowerLevel(this.calculateVelocityModePowerSetting(ElevatorController.OVERRIDE));
            }
            else
            {
                this.component.setMotorPowerLevel(ElevatorController.OVERRIDE);
            }
        }
        else
        {
            // if usePID is true, calculate power-level using PID
            if (this.usePID)
            {
                if (this.useVelocityPID)
                {
                    this.useVelocityPID = false;
                    this.createPIDHandler();
                }

                //calculate position to set elevator
                this.position = this.getPositionShift();

                // reset in case position is less than minimum, or more then maximum
                this.position = Math.max(this.position, ElevatorController.MINIMUM_HEIGHT);
                this.position = Math.min(this.position, ElevatorController.MAXIMUM_HEIGHT);

                this.component.setMotorPowerLevel(this.calculatePositionModePowerSetting(this.position));
            }
            else
            {
                // if we are in non-PID mode, pressing neither the up nor down override buttons means we should stop applying power to the motor
                this.component.setMotorPowerLevel(0.0);
            }
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
            return ElevatorController.TOTE_0 + this.state;
        }
        else if (this.driver.getElevatorMoveTo1ToteButton())
        {
            return ElevatorController.TOTE_1 + this.state;
        }
        else if (this.driver.getElevatorMoveTo2TotesButton())
        {
            return ElevatorController.TOTE_2 + this.state;
        }
        else if (this.driver.getElevatorMoveTo3TotesButton())
        {
            return ElevatorController.TOTE_3 + this.state;
        }
        else
        {
            return this.position;
        }
    }

    @Override
    public void stop()
    {
        // stop the elevator's motor
        this.component.setMotorPowerLevel(0.0);
    }

    private void createPIDHandler()
    {
        if (!this.usePID)
        {
            this.pidHandler = null;
        }
        else
        {
            Preferences prefs = Preferences.getInstance();
            if (!this.useVelocityPID)
            {
                this.pidHandler = new PIDHandler(
                    prefs.getDouble(
                        ElevatorTuningConstants.ELEVATOR_POSITION_PID_KP_KEY,
                        ElevatorTuningConstants.ELEVATOR_POSITION_PID_KP_DEFAULT),
                    prefs.getDouble(
                        ElevatorTuningConstants.ELEVATOR_POSITION_PID_KI_KEY,
                        ElevatorTuningConstants.ELEVATOR_POSITION_PID_KI_DEFAULT),
                    prefs.getDouble(
                        ElevatorTuningConstants.ELEVATOR_POSITION_PID_KD_KEY,
                        ElevatorTuningConstants.ELEVATOR_POSITION_PID_KD_DEFAULT),
                    prefs.getDouble(
                        ElevatorTuningConstants.ELEVATOR_POSITION_PID_KF_KEY,
                        ElevatorTuningConstants.ELEVATOR_POSITION_PID_KF_DEFAULT),
                    ElevatorController.POWERLEVEL_MIN,
                    ElevatorController.POWERLEVEL_MAX);
            }
            else
            {
                this.pidHandler = new PIDHandler(
                    prefs.getDouble(
                        ElevatorTuningConstants.ELEVATOR_VELOCITY_PID_KP_KEY,
                        ElevatorTuningConstants.ELEVATOR_VELOCITY_PID_KP_DEFAULT),
                    prefs.getDouble(
                        ElevatorTuningConstants.ELEVATOR_VELOCITY_PID_KI_KEY,
                        ElevatorTuningConstants.ELEVATOR_VELOCITY_PID_KI_DEFAULT),
                    prefs.getDouble(
                        ElevatorTuningConstants.ELEVATOR_VELOCITY_PID_KD_KEY,
                        ElevatorTuningConstants.ELEVATOR_VELOCITY_PID_KD_DEFAULT),
                    prefs.getDouble(
                        ElevatorTuningConstants.ELEVATOR_VELOCITY_PID_KF_KEY,
                        ElevatorTuningConstants.ELEVATOR_VELOCITY_PID_KF_DEFAULT),
                    ElevatorController.POWERLEVEL_MIN,
                    ElevatorController.POWERLEVEL_MAX);
            }
        }
    }

    private double calculatePositionModePowerSetting(double desired)
    {
        double current = this.component.getEncoderDistance();
        return this.pidHandler.calculate(desired, current);
    }

    private double calculateVelocityModePowerSetting(double desired)
    {
        double current = this.component.getEncoderVelocity();
        return this.pidHandler.calculate(desired, current);
    }
}
