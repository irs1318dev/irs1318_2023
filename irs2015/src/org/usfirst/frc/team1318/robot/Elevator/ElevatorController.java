package org.usfirst.frc.team1318.robot.Elevator;

import org.usfirst.frc.team1318.robot.Common.IController;
import org.usfirst.frc.team1318.robot.Common.IDriver;
import org.usfirst.frc.team1318.robot.Common.PIDHandler;

import edu.wpi.first.wpilibj.Preferences;

public class ElevatorController implements IController
{

    private PIDHandler handler;
    private boolean useVelocityPID = false;
    private boolean usePID;
    private double state;

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

    private static final double OVERRIDE = -1.0;
    private static final double MINIMUM_HEIGHT = 0.0;

    public ElevatorController(ElevatorComponent component, IDriver driver)
    {
        this.component = component;
        this.driver = driver;
        this.createPIDHandler();
        usePID = true;
        state = FLOOR;
    }

    @Override
    public void update()
    {
        // TODO figure out whether it should be moving 
        // TODO figure out which type of PID to use 
        // TODO figure out the target position or velocity 

        // if elevator up or down button is pushed, do not deal with elevator buttons
        if (driver.getElevatorDownButton())
        {
            // if usePID is true, calculate velocity using PID.
            if (usePID)
            {
                component.setMotorVelocity(calculateVelocityModePowerSetting(-OVERRIDE));
            }
            else
            {
                component.setMotorVelocity(-OVERRIDE);
            }
        }
        else if (driver.getElevatorUpButton())
        {
            // if usePID is true, calculate velocity using PID.
            if (usePID)
            {
                component.setMotorVelocity(calculateVelocityModePowerSetting(OVERRIDE));
            }
            else
            {
                component.setMotorVelocity(OVERRIDE);
            }
        }
        else
        {
            // set elevator state here
            if (driver.getElevatorSetStateToFloorButton())
            {
                state = FLOOR;
            }
            else if (driver.getElevatorSetStateToPlatformButton())
            {
                state = PLATFORM;
            }
            else if (driver.getElevatorSetStateToStepButton())
            {
                state = STEP;
            }
            //calculate position to set elevator
            double position = getPositionShift();

            // if position is less than minimum, do not move
            if (position >= MINIMUM_HEIGHT)
            {
                //if usePID is true, calculate velocity using PID
                if (usePID)
                {
                    component.setMotorVelocity(calculatePositionModePowerSetting(position));
                }
                else
                {
                    /**
                     * TODO figure out what to do when usePID is false and position of 
                     * elevator must be set using buttons
                     **/
                }
            }
        }
    }

    private double getPositionShift()
    {
        if (driver.getElevatorMoveTo0TotesButton())
        {
            return TOTE_0 + state;
        }
        else if (driver.getElevatorMoveTo1ToteButton())
        {
            return TOTE_1 + state;
        }
        else if (driver.getElevatorMoveTo2TotesButton())
        {
            return TOTE_2 + state;
        }
        else if (driver.getElevatorMoveTo3TotesButton())
        {
            return TOTE_3 + state;
        }
        else
        {
            // return -1 for no position shift
            return MINIMUM_HEIGHT - 1;
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
