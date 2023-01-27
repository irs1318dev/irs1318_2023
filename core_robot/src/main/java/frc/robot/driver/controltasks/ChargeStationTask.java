//Jamie Hsieh, Calvin Rodrigue
//1.19.2023
//Overshoots the robot forwards on the charge station.
//Jamie's first robot code commit.

package frc.robot.driver.controltasks;

import com.google.inject.Injector;

import frc.robot.TuningConstants;
import frc.robot.TuningConstants.*;
import frc.robot.common.LoggingManager;
import frc.robot.common.robotprovider.ITimer;
import frc.robot.driver.*;
import frc.robot.driver.common.Driver;
import frc.robot.mechanisms.DriveTrainMechanism;
import frc.robot.mechanisms.PigeonManager;

public class ChargeStationTask extends ControlTaskBase
{
    private enum State
    {
        ReadPitch,
        MoveForward,
        MoveBackward,

    }

    private PigeonManager imuManager;
    private DriveTrainMechanism driveTrain;
    private LoggingManager logger;
    private ITimer timer;

    private double pitch;
    private double sign;
    private double xPos;
    private double balancePos;
    private double distanceTraveled;
    private double timeSinceLastPitchLog;
    private double prevPitchLogTime;
    private double[] pitchLog = new double[10];
    private int logRefPoint = 3; //array space in pitchLog[] to reference from for diff

    public ChargeStationTask()
    {
        this.pitch = 0.0;
        this.sign = 1.0;
        this.xPos = 1.0;
        this.balancePos = 0.0;
        this.distanceTraveled = 0.0;
        this.timeSinceLastPitchLog = 0.0;
        this.prevPitchLogTime = 0.0;
        
        
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
        this.timer = this.getInjector().getInstance(ITimer.class);
        this.pitch = this.imuManager.getPitch();
        this.prevPitchLogTime = this.timer.get();
        timeSinceLastPitchLog = this.timer.get() - this.prevPitchLogTime;

        //at the beginning of task set all array values to be current pitch
        for (int i = 0; i < pitchLog.length; i++)
            {
                pitchLog[i] = this.pitch;
            }
        
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

        this.timeSinceLastPitchLog = this.timer.get() - this.prevPitchLogTime; //current time - previous pitch log time

        if (this.timeSinceLastPitchLog >= 0.05)
        {
            //if previous pitch log time was 0.1 secs ago then set pitch log time as current time, and log
            this.prevPitchLogTime = this.timer.get(); 

            //if log position is at max, reset (only log set amount of time)
            //move every value left 1
            for (int i = 1; i < pitchLog.length; i++)
            {
                pitchLog[i-1] = pitchLog[i];
            }
            
            //put current pitch in final space
            pitchLog[pitchLog.length - 1] = this.pitch;
            
        }

        

        //check for IMU pitch diff
        
        
        this.pitch = imuManager.getPitch();
        if (this.pitch <= (TuningConstants.CHARGE_STATION_LEVEL_ANGLE - TuningConstants.CHARGE_STATION_PITCH_VARIATION))
        {
            this.setAnalogOperationState(AnalogOperation.DriveTrainMoveForward, 0.05);
        }
        
        /*
        else if (this.pitch >= (TuningConstants.CHARGE_STATION_LEVEL_ANGLE - TuningConstants.CHARGE_STATION_PITCH_VARIATION))
        {
            this.setAnalogOperationState(AnalogOperation.DriveTrainMoveForward, -0.05);
        }

        previously used for attempting to level on the charge station. Current task does not need to go backwards.
        */ 

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
        //if(Math.abs(pitch) < 0) Previously used for balancing
        if (pitch > 0) //Detects when overshoot
        {
            return true;
        }

        return false;
    }
}