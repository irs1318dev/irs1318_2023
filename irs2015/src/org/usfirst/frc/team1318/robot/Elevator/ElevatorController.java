package org.usfirst.frc.team1318.robot.Elevator;

import org.usfirst.frc.team1318.robot.Common.IController;
import org.usfirst.frc.team1318.robot.Common.IDriver;
import org.usfirst.frc.team1318.robot.Common.PIDHandler;

import edu.wpi.first.wpilibj.Preferences;

public class ElevatorController implements IController
{

    private PIDHandler handler;
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
        //this.createPIDHandler();
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

        if (this.driver.getElevatorPIDOn())
        {
            this.usePID = true;
        }
        if (this.driver.getElevatorPIDOff())
        {
            this.usePID = false;
        }

        // if elevator up or down button is pushed, do not deal with elevator buttons
        if (this.driver.getElevatorDownButton())
        {
            // if usePID is true, calculate velocity using PID.
            if (this.usePID)
            {
                this.useVelocityPID = true;
                this.createPIDHandler();
                this.component.setMotorVelocity(this.calculateVelocityModePowerSetting(-ElevatorController.OVERRIDE));
            }
            else
            {
                this.component.setMotorVelocity(-ElevatorController.OVERRIDE);
            }
        }
        else if (this.driver.getElevatorUpButton())
        {
            // if usePID is true, calculate velocity using PID.
            if (this.usePID)
            {
                this.useVelocityPID = true;
                this.createPIDHandler();
                this.component.setMotorVelocity(this.calculateVelocityModePowerSetting(ElevatorController.OVERRIDE));
            }
            else
            {
                this.component.setMotorVelocity(ElevatorController.OVERRIDE);
            }
        }
        else
        {

            //calculate position to set elevator
            position = this.getPositionShift();

            // if position is less than minimum, do not move
            if (position >= ElevatorController.MINIMUM_HEIGHT && position <= ElevatorController.MAXIMUM_HEIGHT)
            {
                //if usePID is true, calculate velocity using PID
                if (this.usePID)
                {
                    this.useVelocityPID = false;
                    this.createPIDHandler();
                    this.component.setMotorVelocity(this.calculatePositionModePowerSetting(position));
                }
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
            return position;
        }

    }

    @Override
    public void stop()
    {
        // TODO Auto-generated method stub
    }

    private void createPIDHandler()
    {
        Preferences prefs = Preferences.getInstance();
        if (!this.useVelocityPID)
        {
            this.handler = new PIDHandler(
                prefs.getDouble(
                    ElevatorTurningConstants.ELEVATOR_POSITION_PID_KP_KEY,
                    ElevatorTurningConstants.ELEVATOR_POSITION_PID_KP_DEFAULT),
                prefs.getDouble(
                    ElevatorTurningConstants.ELEVATOR_POSITION_PID_KI_KEY,
                    ElevatorTurningConstants.ELEVATOR_POSITION_PID_KI_DEFAULT),
                prefs.getDouble(
                    ElevatorTurningConstants.ELEVATOR_POSITION_PID_KD_KEY,
                    ElevatorTurningConstants.ELEVATOR_POSITION_PID_KD_DEFAULT),
                prefs.getDouble(
                    ElevatorTurningConstants.ELEVATOR_POSITION_PID_KF_KEY,
                    ElevatorTurningConstants.ELEVATOR_POSITION_PID_KF_DEFAULT),
                ElevatorController.POWERLEVEL_MIN,
                ElevatorController.POWERLEVEL_MAX);
        }
        else
        {
            this.handler = new PIDHandler(
                prefs.getDouble(
                    ElevatorTurningConstants.ELEVATOR_VELOCITY_PID_KP_KEY,
                    ElevatorTurningConstants.ELEVATOR_VELOCITY_PID_KP_DEFAULT),
                prefs.getDouble(
                    ElevatorTurningConstants.ELEVATOR_VELOCITY_PID_KI_KEY,
                    ElevatorTurningConstants.ELEVATOR_VELOCITY_PID_KI_DEFAULT),
                prefs.getDouble(
                    ElevatorTurningConstants.ELEVATOR_VELOCITY_PID_KD_KEY,
                    ElevatorTurningConstants.ELEVATOR_VELOCITY_PID_KD_DEFAULT),
                prefs.getDouble(
                    ElevatorTurningConstants.ELEVATOR_VELOCITY_PID_KF_KEY,
                    ElevatorTurningConstants.ELEVATOR_VELOCITY_PID_KF_DEFAULT),
                ElevatorController.POWERLEVEL_MIN,
                ElevatorController.POWERLEVEL_MAX);
        }
    }

    private double calculatePositionModePowerSetting(double desired)
    {
        double current = this.component.getEncoderDistance();
        return this.handler.calculate(desired, current);
    }

    private double calculateVelocityModePowerSetting(double desired)
    {
        double current = this.component.getEncoderVelocity();
        return this.handler.calculate(desired, current);
    }
}
