package org.usfirst.frc.team1318.robot.Elevator;

import org.usfirst.frc.team1318.robot.Common.IController;
import org.usfirst.frc.team1318.robot.Common.IDriver;
import org.usfirst.frc.team1318.robot.Common.PIDHandler;

import edu.wpi.first.wpilibj.Preferences;

public class ElevatorController implements IController
{

    private PIDHandler handler;
    private boolean useVelocityPID = false;

    private final ElevatorComponent component;
    private final IDriver driver;

    public static final double POWERLEVEL_MIN = -.5;
    public static final double POWERLEVEL_MAX = .5;

    private static final double HEIGHT_0 = -1;
    private static final double HEIGHT_1 = -1;
    private static final double HEIGHT_2 = -1;
    private static final double HEIGHT_3 = -1;
    private static final double HEIGHT_4 = -1;
    private static final double HEIGHT_5 = -1;
    private static final double HEIGHT_6 = -1;
    private static final double HEIGHT_7 = -1;

    public ElevatorController(ElevatorComponent component, IDriver driver)
    {
        this.component = component;
        this.driver = driver;
        this.createPIDHandler();
    }

    @Override
    public void update()
    {
        // TODO figure out whether it should be moving 
        // TODO figure out which type of PID to use 
        // TODO figure out the target position or velocity 
        if (this.driver.getElevatorHeight0Button())
        {
            this.component.setMotorVelocity(this.calculatePositionModePowerSetting(ElevatorController.HEIGHT_0));
        }
        else if (this.driver.getElevatorHeight1Button())
        {
            this.component.setMotorVelocity(this.calculatePositionModePowerSetting(ElevatorController.HEIGHT_1));
        }
        else if (this.driver.getElevatorHeight2Button())
        {
            this.component.setMotorVelocity(this.calculatePositionModePowerSetting(ElevatorController.HEIGHT_2));
        }
        else if (this.driver.getElevatorHeight3Button())
        {
            this.component.setMotorVelocity(this.calculatePositionModePowerSetting(ElevatorController.HEIGHT_3));
        }
        else if (this.driver.getElevatorHeight4Button())
        {
            this.component.setMotorVelocity(this.calculatePositionModePowerSetting(ElevatorController.HEIGHT_4));
        }
        else if (this.driver.getElevatorHeight5Button())
        {
            this.component.setMotorVelocity(this.calculatePositionModePowerSetting(ElevatorController.HEIGHT_5));
        }
        else if (this.driver.getElevatorHeight6Button())
        {
            this.component.setMotorVelocity(this.calculatePositionModePowerSetting(ElevatorController.HEIGHT_6));
        }
        else if (this.driver.getElevatorHeight7Button())
        {
            this.component.setMotorVelocity(this.calculatePositionModePowerSetting(ElevatorController.HEIGHT_7));
        }
        else
        {
            this.useVelocityPID = true;
            this.createPIDHandler();
            this.component.setMotorVelocity(this.calculateVelocityModePowerSetting(this.driver.getElevatorOverride()));
            this.useVelocityPID = false;
            this.createPIDHandler();
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
