//Jamie Hsieh, Calvin Rodrigue
//1.19.2023
//Levels the robot on charge station.
//Jamie's first robot code commit.

package frc.robot.driver.controltasks;

import com.google.inject.Injector;

import frc.robot.TuningConstants;
import frc.robot.TuningConstants.*;
import frc.robot.common.LoggingManager;
import frc.robot.driver.*;
import frc.robot.mechanisms.DriveTrainMechanism;
import frc.robot.mechanisms.PigeonManager;


public class ChargeStationTask extends ControlTaskBase
{
    private PigeonManager imuManager;
    private DriveTrainMechanism driveTrain;
    private LoggingManager logger;

    private double pitch;
    private double sign;
    private double xPos;
    private double balancePos;
    private double distanceTraveled;

    public ChargeStationTask()
    {
        this.pitch = 0.0;
        this.sign = 1;
        this.xPos = 1;
        this.balancePos = 0;
        this.distanceTraveled = 0;
    }

    /**
     * Begin the current task.
     */
    @Override
    public void begin()
    {
        this.imuManager = this.getInjector().getInstance(PigeonManager.class);
        this.driveTrain = this.getInjector().getInstance(DriveTrainMechanism.class);
        this.logger = this.getInjector().getInstance(LoggingManager.class);

        this.pitch = this.imuManager.getPitch();

        this.setDigitalOperationState(DigitalOperation.DriveTrainEnableMaintainDirectionMode, true);
        this.setDigitalOperationState(DigitalOperation.DriveTrainPathMode, false);
        this.setAnalogOperationState(AnalogOperation.DriveTrainTurnAngleGoal, 0);
        this.setAnalogOperationState(AnalogOperation.DriveTrainMoveRight, 0);
        this.setAnalogOperationState(AnalogOperation.DriveTrainMoveForward, 0);
        
    }

    /**
     * Run an iteration of the current task.
     */
    @Override
    public void update()
    {
        
        this.pitch = imuManager.getPitch();
        if (this.pitch <= (TuningConstants.CHARGE_STATION_LEVEL_ANGLE - TuningConstants.CHARGE_STATION_PITCH_VARIATION))
        {
            this.setAnalogOperationState(AnalogOperation.DriveTrainMoveForward, 0.2);

            //IF ON 2022 DRIVETRAIN
            //this.setAnalogOperationState(AnalogOperation.DriveTrainMoveRight, 0.5);
        }
        else if (this.pitch >= (TuningConstants.CHARGE_STATION_LEVEL_ANGLE + TuningConstants.CHARGE_STATION_PITCH_VARIATION))
        {
            this.setAnalogOperationState(AnalogOperation.DriveTrainMoveForward, -0.2);
            
            //IF ON 2022 DRIVETRAIN
            //this.setAnalogOperationState(AnalogOperation.DriveTrainMoveRight, -0.5);
        }
        else 
        {
            this.setAnalogOperationState(AnalogOperation.DriveTrainMoveForward, 0);

        }
        
        /*
        if(this.pitch > 0)
        {
            this.sign = 1;
        }
        else if(this.pitch < 0)
        {
            this.sign = -1;
        }       
        this.xPos = driveTrain.getPositionX();
        this.setAnalogOperationState(AnalogOperation.DriveTrainMoveForward, sign * 2 / xPos);
        */


    }

    /**
     * Ends the current task, called when it (or a master task) has completed.
     */
    @Override
    public void end()
    {
        this.setDigitalOperationState(DigitalOperation.DriveTrainEnableMaintainDirectionMode, false);
        this.setDigitalOperationState(DigitalOperation.DriveTrainPathMode, false);
        this.setAnalogOperationState(AnalogOperation.DriveTrainMoveForward, 0);
    }


    /**
     * Checks whether this task has completed, or whether it should continue being processed.
     * @return true if we should continue onto the next task, otherwise false (to keep processing this task)
     */
    @Override
    public boolean hasCompleted()
    {
        if(Math.abs(pitch) < TuningConstants.CHARGE_STATION_PITCH_VARIATION) {//TODO: test if this value is acceptable
            return true;
        }
        return false;
    }

    
}
