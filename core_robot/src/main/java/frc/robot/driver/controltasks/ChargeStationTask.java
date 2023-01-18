package frc.robot.driver.controltasks;

import com.google.inject.Injector;

import frc.robot.TuningConstants;
import frc.robot.TuningConstants.*;
import frc.robot.common.LoggingManager;
import frc.robot.driver.*;
import frc.robot.driver.common.Driver;
import frc.robot.mechanisms.DriveTrainMechanism;
import frc.robot.mechanisms.PigeonManager;


public class ChargeStationTask extends ControlTaskBase
{

    
    private double pitch = 0.0;
    private PigeonManager imuManager;
    private DriveTrainMechanism driveTrain;
    private double sign = 1;
    private double xPos = 1;
    private double balancePos = 0;
    private LoggingManager logger;
    private double distanceTraveled = 0;
    //private Driver driver;

    public ChargeStationTask()
    {
        imuManager = this.getInjector().getInstance(PigeonManager.class);
        driveTrain = this.getInjector().getInstance(DriveTrainMechanism.class);
        logger = this.getInjector().getInstance(LoggingManager.class);
        //this.driver = this.getInjector().getInstance(Driver.class);
    }

    /**
     * Begin the current task.
     */
    @Override
    public void begin()
    {
        
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
            this.setAnalogOperationState(AnalogOperation.DriveTrainMoveForward, 0.5);
        }
        else if (this.pitch >= (TuningConstants.CHARGE_STATION_LEVEL_ANGLE - TuningConstants.CHARGE_STATION_PITCH_VARIATION))
        {
            this.setAnalogOperationState(AnalogOperation.DriveTrainMoveForward, -0.5);
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
        if(Math.abs(pitch) < 0.5) {//TODO: test if this value is acceptable
            return true;
        }
        return false;
    }

    
}
