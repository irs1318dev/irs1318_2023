package org.usfirst.frc.team1318.robot.Elevator;

import org.usfirst.frc.team1318.robot.Common.IController;
import org.usfirst.frc.team1318.robot.Common.IDriver;
import org.usfirst.frc.team1318.robot.Common.PIDHandler;

import edu.wpi.first.wpilibj.Preferences;

public class ElevatorController implements IController
{

    private PIDHandler handler;
    private boolean useVelocityPID = false;

    private ElevatorComponent component;
    private IDriver driver;

    public static final double POWERLEVEL_MIN = -.5;
    public static final double POWERLEVEL_MAX = .5;

    // TODO add constructor 

    @Override
    public void update()
    {
        // TODO figure out whether it should be moving 
        // TODO figure out which type of PID to use 
        // TODO figure out the target position or velocity 

    }

    @Override
    public void stop()
    {
        // TODO Auto-generated method stub

    }

    private void createPIDHandler()
    {
        if (false)
        {
            this.handler = null;
        }
        else
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
