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

    private static final double GROUND = 0;
    private static final double PLATFORM = -1;
    private static final double STEP = -1;

    private static final double TOTE_0 = -1;
    private static final double TOTE_1 = -1;
    private static final double TOTE_2 = -1;
    private static final double TOTE_3 = -1;

    public ElevatorController(ElevatorComponent component, IDriver driver)
    {
        this.component = component;
        this.driver = driver;
        this.createPIDHandler();
        usePID = true;
        state = GROUND;
    }

    @Override
    public void update()
    {
        // TODO figure out whether it should be moving 
        // TODO figure out which type of PID to use 
        // TODO figure out the target position or velocity 

        if (driver.getElevatorSetStateToFloorButton())
        {
            state = GROUND;
        }
        else if (driver.getElevatorSetStateToPlatformButton())
        {
            state = PLATFORM;
        }
        else if (driver.getElevatorSetStateToStepButton())
        {
            state = STEP;
        }

        double position = getPositionShift();
    }

    private double getPositionShift()
    {
        return 0;
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
